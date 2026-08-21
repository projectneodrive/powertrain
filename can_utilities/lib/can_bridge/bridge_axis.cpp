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

bool Axis::idle() {
  // Remembered so an unexplained IDLE can be told apart from one we asked for.
  // "It went idle for no reason" is only answerable if the log knows which of
  // the two ends made it happen.
  _last_disarm_cmd_ms = millis();
  return setAxisState(odcan::AXIS_IDLE);
}
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
void Axis::onFrame(const cansimple::Frame& f, uint32_t now_ms) {
  if (f.node != _link.targetNode()) {
    return;   // another node's traffic; already traced by the diagnostics tap
  }

  switch (f.cmd) {
    // Generated from the firmware's cyclic broadcast list.
#define CAN_RX(cmd, handler)
#define CAN_TX_CYCLIC(cmd, period_ms, sender) \
    case odcan::cmd: rx_##sender(f.data, f.len, now_ms); break;
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

void Axis::rx_sendHeartbeat(const uint8_t* b, uint8_t len, uint32_t now_ms) {
  if (len < 5) return;
  const uint32_t error = getU32(b);
  const uint8_t  state = b[4];

  // THE SCAN'S clock, not millis(). See onFrame() in the header: sampling the
  // time here put last_heartbeat_ms ahead of the `now` the timeouts are checked
  // against, and the unsigned subtraction turned a few milliseconds of skew into
  // 4.29 billion.
  const uint32_t now = now_ms;
  if (_hb_gap_valid) {
    const uint32_t gap = now - _hb_prev_ms;
    if (gap > _hb_max_window)  _hb_max_window  = gap;
    if (gap > _hb_max_session) _hb_max_session = gap;
  }
  _hb_prev_ms       = now;
  _hb_gap_valid     = true;
  _stall_credit_ms  = 0;   // in contact again; the allowance is earned back

  _st.link_up   = true;
  _ever_linked  = true;
  // Re-arm the one-shot: a link that drops, comes back and drops again stops
  // the motor both times.
  _stop_sent          = false;
  _st.safety_stopped  = false;
  if (_link_changed(true)) {
    _hb_max_session = 0;   // a fresh connection gets a fresh measurement
    LOG_I("LINK", "established with node %u", (unsigned)_link.targetNode());
  }

  // Two separate edges, because they are two separate pieces of news at two
  // different severities: an operator arming the motor expects the state line
  // and should not have to hunt for an error bit inside it.
  if (_state_changed(state)) {
    if (_state_changed.first()) {
      LOG_I("AXIS", "state %s", axisnames::state(state));
    } else if (state == odcan::AXIS_IDLE &&
               _state_changed.previous() == odcan::AXIS_CLOSED_LOOP) {
      // Losing closed loop is the transition that matters, and the first
      // question is always "did I do that?". Answer it in the line itself.
      const bool ours = (now - _last_disarm_cmd_ms) < kDisarmAttributionMs;
      if (ours) {
        LOG_I("AXIS", "state CLOSED_LOOP -> IDLE (disarmed by this station)");
      } else {
        LOG_W("AXIS", "state CLOSED_LOOP -> IDLE - NOT commanded here. The board "
                      "disarmed itself: check its own console for a fault, or "
                      "CFG_WATCHDOG_MS if a CAN setpoint timeout is configured");
      }
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
  _st.m.last_heartbeat_ms = now;
}

void Axis::rx_sendEncoderEstimates(const uint8_t* b, uint8_t len, uint32_t now_ms) {
  if (len < 8) return;
  _st.m.pos_rad      = getF32(b)     * TWO_PI;   // rev   -> rad
  _st.m.vel_rad_s    = getF32(b + 4) * TWO_PI;   // rev/s -> rad/s
  _st.m.have_encoder = true;
}

void Axis::rx_sendIq(const uint8_t* b, uint8_t len, uint32_t now_ms) {
  if (len < 8) return;
  _st.m.iq_setpoint_a = getF32(b);
  _st.m.iq_measured_a = getF32(b + 4);
  _st.m.have_iq       = true;
}

void Axis::rx_sendBusVI(const uint8_t* b, uint8_t len, uint32_t now_ms) {
  if (len < 8) return;
  _st.m.vbus_v     = getF32(b);
  _st.m.ibus_a     = getF32(b + 4);
  _st.m.have_vbus  = true;
}

// ---------------------------------------------------------------------------
uint32_t Axis::heartbeatAge(uint32_t now_ms) const {
  // Signed difference, floored at zero. A heartbeat can legitimately carry a
  // timestamp a hair ahead of the scan clock (creditStall moves the reference,
  // and clocks are sampled at different points); as an unsigned subtraction
  // that reads as ~49 days of silence instead of "no time has passed".
  const int32_t age = (int32_t)(now_ms - _st.m.last_heartbeat_ms);
  return age > 0 ? (uint32_t)age : 0u;
}

bool Axis::refreshLink(uint32_t now_ms) {
  const bool fresh = heartbeatAge(now_ms) < kLinkTimeoutMs;
  if (_st.link_up && !fresh) {
    _st.link_up = false;
    ++_link_drops;
    if (_link_changed(false)) {
      // The drop count and the worst gap go in the message as well as on the
      // status line: when this fires repeatedly, the log is where somebody is
      // looking, and these two numbers are the whole diagnosis.
      //
      // While the link is up a gap can never reach kLinkTimeoutMs — exceeding
      // it is what declares the loss. So the worst gap says how ragged the link
      // was *while it worked*: near the heartbeat period means it was clean
      // right up to the moment it stopped (a sender that died), well above
      // means frames were already being lost (a wire that is marginal).
      LOG_W("LINK", "lost with node %u (silent for %lums; drop #%lu; worst gap while "
                    "up %lums vs %lums expected -> %s)",
            (unsigned)_link.targetNode(), (unsigned long)kLinkTimeoutMs,
            (unsigned long)_link_drops, (unsigned long)_hb_max_session,
            (unsigned long)kHeartbeatPeriodMs,
            (_hb_max_session > kHeartbeatPeriodMs * 2)
                ? "frames were already being lost, suspect wiring/termination"
                : "the link was clean, so the sender stopped (board reset? bus cut?)");
    }
    // The silence that follows is the outage, not a heartbeat gap.
    _hb_gap_valid = false;
  }
  return fresh;
}

void Axis::creditStall(uint32_t stall_ms, uint32_t now_ms) {
  // BOUNDED, and the bound is the safety-critical part.
  //
  // Crediting back time we were blind is right: silence nobody listened to is
  // not evidence the board went quiet. But applied without a limit it is a way
  // to never disarm at all — a station stalling repeatedly would push the
  // reference forward every time, the age would never reach the threshold, and
  // a motor running with a genuinely dead master would keep running.
  //
  // So a single outage may be forgiven at most kLinkLossStopMs in total. Past
  // that the stop fires regardless: at worst it is late by that much, which is
  // the right way round to be wrong.
  if (_stall_credit_ms >= kLinkLossStopMs) return;
  const uint32_t room  = kLinkLossStopMs - _stall_credit_ms;
  const uint32_t grant = (stall_ms > room) ? room : stall_ms;
  _stall_credit_ms += grant;

  // Clamped to now so the reference can never end up in the future.
  const uint32_t credited = _st.m.last_heartbeat_ms + grant;
  _st.m.last_heartbeat_ms =
      ((int32_t)(credited - now_ms) > 0) ? now_ms : credited;

  // Whatever arrives next is not one inter-frame gap: frames queued during the
  // stall, and any the TWAI queue dropped, are both in there. Measuring it
  // would report this station's stall as the bus being ragged.
  _hb_gap_valid = false;
}

uint32_t Axis::takeMaxHeartbeatGap() {
  const uint32_t gap = _hb_max_window;
  _hb_max_window = 0;
  return gap;
}

void Axis::checkLinkLossStop(uint32_t now_ms) {
  if (kLinkLossStopMs == 0) return;          // disabled in bridge_config.h
  if (!_ever_linked || _stop_sent) return;   // no falling edge, or already fired
  if (heartbeatAge(now_ms) < kLinkLossStopMs) return;

  // Latch BEFORE sending. If the transmit fails — which is likely, the bus is
  // the thing that just died — we must not retry it every scan: that is the
  // command flood this one-shot exists to avoid, and it would make the console
  // unusable at exactly the moment somebody is trying to use it.
  _stop_sent           = true;
  _st.safety_stopped   = true;
  _last_disarm_cmd_ms  = now_ms;   // so the resulting IDLE is attributed to us

  // Disarm rather than commanding zero velocity. Holding 0 rad/s keeps the axis
  // armed and actively braking, which is the opposite of safe on a link that
  // just vanished; IDLE lets it coast. It is also what the firmware's own CAN
  // watchdog does (CFG_WATCHDOG_MS in motor_config.h).
  //
  // The cached setpoint is deliberately NOT zeroed: it still records what we
  // last commanded, which is the truth. Re-arming with 'A' zeroes it anyway.
  if (idle()) {
    LOG_W("LINK", "no heartbeat for %lums - axis DISARMED (one-shot safety stop). "
                  "Commands still go out; 'A' re-arms once the link is back.",
          (unsigned long)kLinkLossStopMs);
  } else {
    LOG_E("LINK", "no heartbeat for %lums and the safety stop could NOT be sent - "
                  "the bus is down, the axis is on its own watchdog now "
                  "(CFG_WATCHDOG_MS in motor_config.h)",
          (unsigned long)kLinkLossStopMs);
  }
}

}  // namespace bridge
