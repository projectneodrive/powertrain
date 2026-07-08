// ============================================================================
//  odrive_can.h  —  ODrive CANSimple protocol layer for the SimpleFOC firmware.
//
//  Speaks the ODrive "CANSimple" protocol so the existing DBC
//  (CAN/create_can_dbc.py) and odrivetool / any CAN master work unchanged.
//  Arbitration id = (node_id << 5) | cmd_id  (11-bit standard, little-endian).
//
//  This module is deliberately decoupled from SimpleFOC: it only reads/writes a
//  shared AxisIO block. main.cpp bridges AxisIO <-> the BLDCMotor. Runs on
//  pazi88/STM32_CAN (CAN1 on PB8/PB9).
// ============================================================================
#pragma once
#include <Arduino.h>
#include <STM32_CAN.h>

namespace odcan {

// ---- CANSimple command ids (low 5 bits of the arbitration id) --------------
enum Cmd : uint8_t {
  CMD_HEARTBEAT             = 0x001,
  CMD_ESTOP                 = 0x002,
  CMD_GET_MOTOR_ERROR       = 0x003,
  CMD_GET_ENCODER_ERROR     = 0x004,
  CMD_GET_SENSORLESS_ERROR  = 0x005,
  CMD_SET_AXIS_NODE_ID      = 0x006,
  CMD_SET_AXIS_STATE        = 0x007,
  CMD_GET_ENCODER_ESTIMATES = 0x009,
  CMD_GET_ENCODER_COUNT     = 0x00A,
  CMD_SET_CONTROLLER_MODE   = 0x00B,
  CMD_SET_INPUT_POS         = 0x00C,
  CMD_SET_INPUT_VEL         = 0x00D,
  CMD_SET_INPUT_TORQUE      = 0x00E,
  CMD_SET_LIMITS            = 0x00F,
  CMD_GET_IQ                = 0x014,
  CMD_GET_SENSORLESS_EST    = 0x015,
  CMD_REBOOT                = 0x016,
  CMD_GET_BUS_VI            = 0x017,
  CMD_CLEAR_ERRORS          = 0x018,
  CMD_SET_LINEAR_COUNT      = 0x019,
  CMD_SET_POS_GAIN          = 0x01A,
  CMD_SET_VEL_GAINS         = 0x01B,
  CMD_GET_ADC_VOLTAGE       = 0x01C,
  CMD_GET_CONTROLLER_ERROR  = 0x01D,
};

// ---- ODrive enums (fw-v0.5.6 values) ---------------------------------------
enum AxisState : uint8_t {
  AXIS_UNDEFINED   = 0,
  AXIS_IDLE        = 1,
  AXIS_SENSORLESS  = 5,   // SENSORLESS_CONTROL (fw <= 0.5.1 numbering; see plan)
  AXIS_MOTOR_CAL   = 4,
  AXIS_ENC_OFFSET_CAL = 7,
  AXIS_CLOSED_LOOP = 8,
};
enum ControlMode : uint8_t {
  CTRL_VOLTAGE  = 0,
  CTRL_TORQUE   = 1,
  CTRL_VELOCITY = 2,
  CTRL_POSITION = 3,
};
enum AxisErrorBits : uint32_t {
  ERR_NONE              = 0x00000000,
  ERR_INVALID_STATE     = 0x00000001,
  ERR_MOTOR_FAILED      = 0x00000040,
  ERR_SENSORLESS_FAILED = 0x00000080,
  ERR_ENCODER_FAILED    = 0x00000100,
  ERR_CONTROLLER_FAILED = 0x00000200,
  ERR_WATCHDOG_EXPIRED  = 0x00000800,
  ERR_ESTOP_REQUESTED   = 0x00004000,
};

// ---- Shared command + telemetry block (bridged to SimpleFOC by main.cpp) ----
//  All SI/rad units on the firmware side; CAN conversions (rev/Nm) happen here.
struct AxisIO {
  // commands (CAN writes, control loop reads)
  volatile bool     armed            = false;   // CLOSED_LOOP requested
  volatile bool     estop            = false;   // latched
  volatile uint8_t  control_mode     = CTRL_TORQUE;
  volatile uint8_t  input_mode       = 1;       // PASSTHROUGH
  volatile float    input_pos        = 0.0f;    // rad
  volatile float    input_vel        = 0.0f;    // rad/s
  volatile float    input_torque     = 0.0f;    // Nm (used as Uq volts until Phase 4)
  volatile float    vel_limit        = 0.0f;    // rad/s
  volatile float    current_limit    = 0.0f;    // A
  volatile float    pos_gain         = 0.0f;    // (rad/s)/rad
  volatile float    vel_gain         = 0.0f;    // request; see main mapping note
  volatile float    vel_int_gain     = 0.0f;
  volatile uint32_t last_setpoint_ms = 0;       // watchdog feed
  volatile bool     req_reboot       = false;
  volatile bool     req_clear_errors = false;
  volatile bool     new_mode         = false;   // control_mode changed by CAN

  // telemetry (control loop writes, CAN reads)
  volatile float    pos_rev     = 0.0f;         // rev
  volatile float    vel_rev     = 0.0f;         // rev/s
  volatile float    iq_setpoint = 0.0f;         // A
  volatile float    iq_measured = 0.0f;         // A
  volatile float    vbus        = 0.0f;         // V
  volatile float    ibus        = 0.0f;         // A
  volatile uint32_t axis_error  = 0;            // ORed AxisErrorBits
  volatile uint8_t  cur_state   = AXIS_IDLE;    // current AxisState
  volatile uint32_t motor_error = 0;
  volatile uint32_t encoder_error = 0;
  volatile uint32_t controller_error = 0;
};

// ---- The CAN interface -----------------------------------------------------
class OdriveCAN {
public:
  explicit OdriveCAN(AxisIO& io) : _io(io) {}

  // hardware bring-up: node id, bit rate, and NVIC priority for the CAN IRQ.
  void begin(uint8_t node_id, uint32_t baud, uint8_t irq_prio);

  // drain the RX ring buffer and dispatch (call from the comms task, >=1 kHz).
  void poll();

  // send the cyclic telemetry frames that are due at time `now_ms`.
  void txCyclic(uint32_t now_ms);

private:
  void dispatch(uint8_t cmd, const CAN_message_t& m);
  void send(uint8_t cmd, const uint8_t* d, uint8_t len);
  void sendHeartbeat();
  void sendEncoderEstimates();
  void sendIq();
  void sendBusVI();

  AxisIO&    _io;
  STM32_CAN  _can{CAN1, ALT, RX_SIZE_64, TX_SIZE_16};   // CAN1 on PB8/PB9
  uint8_t    _node = 0;
  uint32_t   _t_hb = 0, _t_enc = 0, _t_iq = 0, _t_vi = 0;
};

} // namespace odcan
