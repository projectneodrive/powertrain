// ============================================================================
//  bridge_axis.cpp — see bridge_axis.h.
// ============================================================================
#include "bridge_axis.h"

namespace bridge {

using cansimple::getF32;
using cansimple::getU32;
using cansimple::putF32;
using cansimple::putU32;

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------
bool Axis::setAxisState(uint8_t state) {
  uint8_t d[8] = {0};
  putU32(d, state);
  return _link.send<odcan::CMD_SET_AXIS_STATE>(d, 8);
}

bool Axis::arm() {
  return clearErrors()
      && setControllerMode(odcan::CTRL_VELOCITY, INPUT_PASSTHROUGH)
      && setVelocity(0.0f)
      && setAxisState(odcan::AXIS_CLOSED_LOOP);
}

bool Axis::idle()         { return setAxisState(odcan::AXIS_IDLE); }
bool Axis::characterise() { return setAxisState(odcan::AXIS_MOTOR_CAL); }

bool Axis::estop()       { return _link.send<odcan::CMD_ESTOP>(); }
bool Axis::clearErrors() { return _link.send<odcan::CMD_CLEAR_ERRORS>(); }
bool Axis::reboot()      { return _link.send<odcan::CMD_REBOOT>(); }

// ---------------------------------------------------------------------------
//  Setpoints
// ---------------------------------------------------------------------------
bool Axis::setControllerMode(uint8_t mode, uint8_t input_mode) {
  uint8_t d[8] = {0};
  putU32(d,     mode);
  putU32(d + 4, input_mode);
  _st.c.control_mode = mode;
  _st.c.input_mode   = input_mode;
  return _link.send<odcan::CMD_SET_CONTROLLER_MODE>(d, 8);
}

bool Axis::setTorque(float nm) {
  uint8_t d[8] = {0};
  putF32(d, nm);                              // Nm, no conversion
  _st.c.control_mode  = odcan::CTRL_TORQUE;
  _st.c.input_torque  = nm;
  _st.c.target        = nm;
  return _link.send<odcan::CMD_SET_INPUT_TORQUE>(d, 8);
}

bool Axis::setVelocity(float rad_s) {
  uint8_t d[8] = {0};
  putF32(d,     rad_s / TWO_PI);              // rad/s -> rev/s
  putF32(d + 4, 0.0f);                        // torque feed-forward, unused
  _st.c.control_mode = odcan::CTRL_VELOCITY;
  _st.c.input_vel    = rad_s;
  _st.c.target       = rad_s;
  return _link.send<odcan::CMD_SET_INPUT_VEL>(d, 8);
}

bool Axis::setPosition(float rad) {
  uint8_t d[8] = {0};
  putF32(d, rad / TWO_PI);                    // rad -> rev
  _st.c.control_mode = odcan::CTRL_POSITION;
  _st.c.input_pos    = rad;
  _st.c.target       = rad;
  return _link.send<odcan::CMD_SET_INPUT_POS>(d, 8);
}

bool Axis::driveVelocity(float rad_s) {
  const bool mode_ok = (_st.c.control_mode == odcan::CTRL_VELOCITY) ||
                       setControllerMode(odcan::CTRL_VELOCITY, INPUT_PASSTHROUGH);
  return mode_ok && setVelocity(rad_s);
}

// ---------------------------------------------------------------------------
//  Configuration
// ---------------------------------------------------------------------------
bool Axis::applyLimits() {
  uint8_t d[8] = {0};
  putF32(d,     _st.c.vel_limit / TWO_PI);    // rad/s -> rev/s
  putF32(d + 4, _st.c.current_limit);         // A
  return _link.send<odcan::CMD_SET_LIMITS>(d, 8);
}

bool Axis::applyVelGains() {
  uint8_t d[8] = {0};
  putF32(d,     _st.c.vel_gain * TWO_PI);     // Nm/(rad/s) -> Nm/(rev/s)
  putF32(d + 4, _st.c.vel_int_gain * TWO_PI);
  return _link.send<odcan::CMD_SET_VEL_GAINS>(d, 8);
}

bool Axis::applyPosGain() {
  uint8_t d[8] = {0};
  putF32(d, _st.c.pos_gain);                  // (rad/s)/rad, no conversion
  return _link.send<odcan::CMD_SET_POS_GAIN>(d, 8);
}

bool Axis::requestErrors() {
  // Zero-length requests; the board answers with the same command id carrying
  // the code, decoded in onReply(). All three are sent unconditionally — if the
  // first fails, the other two are the ones that might still say why.
  const bool a = _link.send<odcan::CMD_GET_MOTOR_ERROR>();
  const bool b = _link.send<odcan::CMD_GET_ENCODER_ERROR>();
  const bool c = _link.send<odcan::CMD_GET_CONTROLLER_ERROR>();
  return a && b && c;
}

// ---------------------------------------------------------------------------
//  Receive
// ---------------------------------------------------------------------------
void Axis::onFrame(const cansimple::Frame& f) {
  if (f.node != _link.targetNode()) {
    return;   // another node's traffic; already traced by the diagnostics tap
  }

  switch (f.cmd) {
    // Generated from the firmware's cyclic broadcast list.
#define CAN_RX(cmd, handler)
#define CAN_TX_CYCLIC(cmd, period_ms, sender) \
    case odcan::cmd: rx_##sender(f.data, f.len); break;
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX

    default:
      onReply(f);
      break;
  }
}

void Axis::onReply(const cansimple::Frame& f) {
  if (f.len < 4) return;
  switch (f.cmd) {
    case odcan::CMD_GET_MOTOR_ERROR:      _st.m.motor_error      = getU32(f.data); break;
    case odcan::CMD_GET_ENCODER_ERROR:    _st.m.encoder_error    = getU32(f.data); break;
    case odcan::CMD_GET_CONTROLLER_ERROR: _st.m.controller_error = getU32(f.data); break;
    default: break;
  }
}

// ---- the cyclic decoders ---------------------------------------------------
// Each is named after the firmware sender whose frame it decodes; the length
// checks are not paranoia, a short frame here would read past the payload.

void Axis::rx_sendHeartbeat(const uint8_t* b, uint8_t len) {
  if (len < 5) return;
  const uint32_t error = getU32(b);
  const uint8_t  state = b[4];

  _st.link_up = true;
  if (_link_changed(true)) {
    LOG_I("LINK", "established with node %u", (unsigned)_link.targetNode());
  }

  // Two separate edges, because they are two separate pieces of news at two
  // different severities: an operator arming the motor expects the state line
  // and should not have to hunt for an error bit inside it.
  if (_state_changed(state)) {
    if (_state_changed.first()) {
      LOG_I("AXIS", "state %s", axisnames::state(state));
    } else {
      LOG_I("AXIS", "state %s -> %s", axisnames::state(_state_changed.previous()),
            axisnames::state(state));
    }
  }
  if (_error_changed(error)) {
    char names[128];
    axisnames::errors(error, names, sizeof(names));
    if (error != 0) {
      // A board that was ALREADY faulted when we connected is reported without
      // a bogus "0x0 ->" prefix; it did not just happen, we just arrived.
      if (_error_changed.first()) {
        LOG_E("AXIS", "error 0x%lX %s (already latched when the link came up)",
              (unsigned long)error, names);
      } else {
        LOG_E("AXIS", "error 0x%lX -> 0x%lX %s", (unsigned long)_error_changed.previous(),
              (unsigned long)error, names);
      }
    } else if (!_error_changed.first()) {
      LOG_I("AXIS", "error cleared (was 0x%lX)", (unsigned long)_error_changed.previous());
    }
  }

  _st.m.axis_error        = error;
  _st.m.heartbeat_state   = state;
  _st.m.last_heartbeat_ms = millis();
}

void Axis::rx_sendEncoderEstimates(const uint8_t* b, uint8_t len) {
  if (len < 8) return;
  _st.m.pos_rad      = getF32(b)     * TWO_PI;   // rev   -> rad
  _st.m.vel_rad_s    = getF32(b + 4) * TWO_PI;   // rev/s -> rad/s
  _st.m.have_encoder = true;
}

void Axis::rx_sendIq(const uint8_t* b, uint8_t len) {
  if (len < 8) return;
  _st.m.iq_setpoint_a = getF32(b);
  _st.m.iq_measured_a = getF32(b + 4);
  _st.m.have_iq       = true;
}

void Axis::rx_sendBusVI(const uint8_t* b, uint8_t len) {
  if (len < 8) return;
  _st.m.vbus_v     = getF32(b);
  _st.m.ibus_a     = getF32(b + 4);
  _st.m.have_vbus  = true;
}

// ---------------------------------------------------------------------------
bool Axis::refreshLink(uint32_t now_ms) {
  const bool fresh = (now_ms - _st.m.last_heartbeat_ms) < kLinkTimeoutMs;
  if (_st.link_up && !fresh) {
    _st.link_up = false;
    if (_link_changed(false)) {
      LOG_W("LINK", "lost with node %u (no heartbeat for %lums)",
            (unsigned)_link.targetNode(), (unsigned long)kLinkTimeoutMs);
    }
  }
  return fresh;
}

}  // namespace bridge
