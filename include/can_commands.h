// Single source of truth for the CANSimple command set.
//
// An X-macro list, like console_commands.h and telemetry_schema.h next to it.
// The includer #defines CAN_RX and CAN_TX_CYCLIC before #including this file
// and #undefs them after; both must be defined, even if to nothing, since every
// expansion sees both kinds.
//
//   CAN_RX(cmd, handler)
//     cmd      command id (low 5 bits of the arbitration id), from odcan::Cmd
//     handler  private OdriveCAN member: void handler(const uint8_t* b)
//              b is the 8-byte payload. Handlers that take no payload ignore it.
//
//   CAN_TX_CYCLIC(cmd, period_ms, sender)
//     cmd        for documentation and to keep the table greppable by id
//     period_ms  transmit interval
//     sender     private OdriveCAN member: void sender()
//
// A command may appear in both lists: the three telemetry getters below answer
// an explicit request AND broadcast on a timer, which is exactly how ODrive
// behaves. Declaring both here is what keeps those two paths from drifting.
//
// Adding a command is one line plus a handler in odrive_can.cpp. Anything not
// listed is silently ignored on RX (anticogging, trajectory moves, etc.),
// which is deliberate: an unknown frame from a richer master must not fault us.

// ---- Setters and triggers ---------------------------------------------------
CAN_RX(CMD_ESTOP,               rxEstop)
CAN_RX(CMD_SET_AXIS_STATE,      rxSetAxisState)
CAN_RX(CMD_SET_CONTROLLER_MODE, rxSetControllerMode)
CAN_RX(CMD_SET_INPUT_POS,       rxSetInputPos)
CAN_RX(CMD_SET_INPUT_VEL,       rxSetInputVel)
CAN_RX(CMD_SET_INPUT_TORQUE,    rxSetInputTorque)
CAN_RX(CMD_SET_LIMITS,          rxSetLimits)
CAN_RX(CMD_SET_POS_GAIN,        rxSetPosGain)
CAN_RX(CMD_SET_VEL_GAINS,       rxSetVelGains)
CAN_RX(CMD_CLEAR_ERRORS,        rxClearErrors)
CAN_RX(CMD_REBOOT,              rxReboot)
CAN_RX(CMD_SET_AXIS_NODE_ID,    rxSetAxisNodeId)

// ---- Getters (answered immediately) -----------------------------------------
CAN_RX(CMD_GET_ENCODER_ESTIMATES, rxGetEncoderEstimates)
CAN_RX(CMD_GET_IQ,                rxGetIq)
CAN_RX(CMD_GET_BUS_VI,            rxGetBusVI)
CAN_RX(CMD_GET_MOTOR_ERROR,       rxGetMotorError)
CAN_RX(CMD_GET_ENCODER_ERROR,     rxGetEncoderError)
CAN_RX(CMD_GET_CONTROLLER_ERROR,  rxGetControllerError)

// ---- Cyclic telemetry -------------------------------------------------------
// Order matters only in that it fixes the slot each timer uses; keep encoder
// estimates first, it is the one with a rate anybody depends on.
CAN_TX_CYCLIC(CMD_GET_ENCODER_ESTIMATES,  10, sendEncoderEstimates)   // 100 Hz
CAN_TX_CYCLIC(CMD_HEARTBEAT,             100, sendHeartbeat)          //  10 Hz
CAN_TX_CYCLIC(CMD_GET_IQ,                100, sendIq)
CAN_TX_CYCLIC(CMD_GET_BUS_VI,            100, sendBusVI)
