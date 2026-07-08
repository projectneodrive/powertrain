// ============================================================================
//  odrive_can.cpp  —  ODrive CANSimple RX dispatch + cyclic TX.
// ============================================================================
#include "odrive_can.h"
#include <string.h>

namespace odcan {

// ---- little-endian pack/unpack (Cortex-M4 is little-endian) -----------------
static inline uint32_t rd_u32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline float    rd_f32(const uint8_t* p) { float f;    memcpy(&f, p, 4); return f; }
static inline void     wr_u32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }
static inline void     wr_f32(uint8_t* p, float f)    { memcpy(p, &f, 4); }

// ---------------------------------------------------------------------------
void OdriveCAN::begin(uint8_t node_id, uint32_t baud, uint8_t irq_prio) {
  _node = node_id;
  _can.begin();
  _can.setBaudRate(baud);
  _can.setIRQPriority(irq_prio, 0);
  // Accept only frames addressed to our node: match the top 6 id bits
  // (node<<5), leave the low 5 (cmd) as don't-care.
  _can.setFilterSingleMask(0, (uint32_t)_node << 5, 0x7E0, STD);
  // Plus the broadcast E-stop (0x002) regardless of node.
  _can.setFilterSingleMask(1, CMD_ESTOP, 0x7FF, STD);
}

// ---------------------------------------------------------------------------
void OdriveCAN::poll() {
  CAN_message_t m;
  while (_can.read(m)) {
    if (m.flags.extended) continue;               // CANSimple uses standard ids
    uint8_t node = (m.id >> 5) & 0x3F;
    uint8_t cmd  = m.id & 0x1F;
    if (node != _node && m.id != CMD_ESTOP) continue;
    dispatch(cmd, m);
  }
}

// ---------------------------------------------------------------------------
void OdriveCAN::dispatch(uint8_t cmd, const CAN_message_t& m) {
  const uint8_t* b = m.buf;
  switch (cmd) {
    // -------- setters / triggers --------
    case CMD_ESTOP:
      _io.estop = true;
      _io.armed = false;
      _io.axis_error |= ERR_ESTOP_REQUESTED;
      break;

    case CMD_SET_AXIS_STATE: {
      uint32_t s = rd_u32(b);
      if (s == AXIS_CLOSED_LOOP || s == AXIS_SENSORLESS) {
        if (!_io.estop) { _io.armed = true; _io.last_setpoint_ms = millis(); }
      } else if (s == AXIS_IDLE) {
        _io.armed = false;
      }
      break;
    }

    case CMD_SET_CONTROLLER_MODE:
      _io.control_mode = (uint8_t)rd_u32(b);
      _io.input_mode   = (uint8_t)rd_u32(b + 4);
      _io.new_mode     = true;
      break;

    case CMD_SET_INPUT_POS:
      _io.input_pos = rd_f32(b) * TWO_PI;          // rev -> rad
      _io.last_setpoint_ms = millis();
      break;

    case CMD_SET_INPUT_VEL:
      _io.input_vel = rd_f32(b) * TWO_PI;          // rev/s -> rad/s
      _io.last_setpoint_ms = millis();             // (torque FF in b+4 ignored)
      break;

    case CMD_SET_INPUT_TORQUE:
      _io.input_torque = rd_f32(b);                // Nm
      _io.last_setpoint_ms = millis();
      break;

    case CMD_SET_LIMITS:
      _io.vel_limit     = rd_f32(b) * TWO_PI;      // rev/s -> rad/s
      _io.current_limit = rd_f32(b + 4);           // A
      break;

    case CMD_SET_POS_GAIN:
      _io.pos_gain = rd_f32(b);
      break;

    case CMD_SET_VEL_GAINS:
      _io.vel_gain     = rd_f32(b);
      _io.vel_int_gain = rd_f32(b + 4);
      break;

    case CMD_CLEAR_ERRORS:
      _io.req_clear_errors = true;
      _io.estop = false;
      break;

    case CMD_REBOOT:
      _io.req_reboot = true;
      break;

    case CMD_SET_AXIS_NODE_ID:
      _node = (uint8_t)rd_u32(b);
      _can.setFilterSingleMask(0, (uint32_t)_node << 5, 0x7E0, STD);
      break;

    // -------- getters (reply immediately) --------
    case CMD_GET_ENCODER_ESTIMATES: sendEncoderEstimates(); break;
    case CMD_GET_IQ:                sendIq();               break;
    case CMD_GET_BUS_VI:            sendBusVI();            break;
    case CMD_GET_MOTOR_ERROR: {
      uint8_t d[8] = {0}; wr_u32(d, _io.motor_error);      send(CMD_GET_MOTOR_ERROR, d, 8); break;
    }
    case CMD_GET_ENCODER_ERROR: {
      uint8_t d[8] = {0}; wr_u32(d, _io.encoder_error);    send(CMD_GET_ENCODER_ERROR, d, 8); break;
    }
    case CMD_GET_CONTROLLER_ERROR: {
      uint8_t d[8] = {0}; wr_u32(d, _io.controller_error); send(CMD_GET_CONTROLLER_ERROR, d, 8); break;
    }
    default:
      break;   // unsupported cmd -> silently ignore (anticogging, traj, etc.)
  }
}

// ---------------------------------------------------------------------------
void OdriveCAN::txCyclic(uint32_t now) {
  if (now - _t_enc >= 10)  { _t_enc = now; sendEncoderEstimates(); }  // 100 Hz
  if (now - _t_hb  >= 100) { _t_hb  = now; sendHeartbeat();        }  // 10 Hz
  if (now - _t_iq  >= 100) { _t_iq  = now; sendIq();               }
  if (now - _t_vi  >= 100) { _t_vi  = now; sendBusVI();            }
}

// ---------------------------------------------------------------------------
void OdriveCAN::send(uint8_t cmd, const uint8_t* d, uint8_t len) {
  CAN_message_t m;
  m.id = ((uint32_t)_node << 5) | cmd;
  m.flags.extended = false;
  m.flags.remote   = false;
  m.len = len;
  for (uint8_t i = 0; i < len && i < 8; i++) m.buf[i] = d[i];
  _can.write(m);
}

void OdriveCAN::sendHeartbeat() {
  uint8_t d[8] = {0};
  wr_u32(d, _io.axis_error);
  d[4] = _io.cur_state;
  d[5] = (_io.axis_error & ERR_MOTOR_FAILED)      ? 1 : 0;
  d[6] = (_io.axis_error & ERR_ENCODER_FAILED)    ? 1 : 0;
  d[7] = (_io.axis_error & ERR_CONTROLLER_FAILED) ? 1 : 0;   // bit7 = traj-done
  send(CMD_HEARTBEAT, d, 8);
}

void OdriveCAN::sendEncoderEstimates() {
  uint8_t d[8];
  wr_f32(d,     _io.pos_rev);
  wr_f32(d + 4, _io.vel_rev);
  send(CMD_GET_ENCODER_ESTIMATES, d, 8);
}

void OdriveCAN::sendIq() {
  uint8_t d[8];
  wr_f32(d,     _io.iq_setpoint);
  wr_f32(d + 4, _io.iq_measured);
  send(CMD_GET_IQ, d, 8);
}

void OdriveCAN::sendBusVI() {
  uint8_t d[8];
  wr_f32(d,     _io.vbus);
  wr_f32(d + 4, _io.ibus);
  send(CMD_GET_BUS_VI, d, 8);
}

} // namespace odcan
