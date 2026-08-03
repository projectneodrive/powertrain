// Single source of truth for the axis vocabulary: states, control modes and
// error bits, together with the SHORT NAMES an operator reads.
//
// An X-macro list, like the other tables in this directory. The includer defines
// AXIS_STATE, AXIS_MODE and AXIS_ERROR before #including it and #undefs them
// after; all three must be defined, even if to nothing.
//
//   AXIS_STATE(name, value, label)   ODrive AxisState
//   AXIS_MODE(name, value, label)    ODrive ControlMode
//   AXIS_ERROR(name, value, label)   one BIT of the axis error word
//
// The `label` column is what makes this a table rather than three enums:
//
//   * gvl/axis_io.h generates the enums from the name/value columns;
//   * can_utilities decodes a heartbeat into "IDLE -> CLOSED_LOOP" and an error
//     word into "[MOTOR_FAILED|ENCODER_FAILED]" instead of printing raw hex;
//   * the web GUI's CAN Devices page decodes the same two things for display.
//
// Without the shared labels each of those three would carry its own copy of the
// names, and a new error bit would show up as an undecoded number in two of
// them. Add a bit HERE and all three learn it.
//
// ERR_NONE is deliberately absent from the AXIS_ERROR list: it is the ABSENCE
// of bits, not a bit, and a decoder walking the list would match it against
// every value. gvl/axis_io.h defines it separately.

// ---- States (ODrive fw-v0.5.6 values) --------------------------------------
AXIS_STATE(AXIS_UNDEFINED,      0, "UNDEFINED")
AXIS_STATE(AXIS_IDLE,           1, "IDLE")
AXIS_STATE(AXIS_MOTOR_CAL,      4, "MOTOR_CALIBRATION")
AXIS_STATE(AXIS_SENSORLESS,     5, "SENSORLESS_CONTROL")
AXIS_STATE(AXIS_ENC_OFFSET_CAL, 7, "ENCODER_OFFSET_CALIBRATION")
AXIS_STATE(AXIS_CLOSED_LOOP,    8, "CLOSED_LOOP")

// ---- Control modes ----------------------------------------------------------
AXIS_MODE(CTRL_VOLTAGE,  0, "VOLTAGE")
AXIS_MODE(CTRL_TORQUE,   1, "TORQUE")
AXIS_MODE(CTRL_VELOCITY, 2, "VELOCITY")
AXIS_MODE(CTRL_POSITION, 3, "POSITION")

// ---- Error bits (ORed into AxisIO::axis_error) ------------------------------
AXIS_ERROR(ERR_INVALID_STATE,       0x00000001, "INVALID_STATE")
AXIS_ERROR(ERR_DC_BUS_OVER_VOLTAGE, 0x00000004, "DC_BUS_OVER_VOLTAGE")
AXIS_ERROR(ERR_MOTOR_FAILED,        0x00000040, "MOTOR_FAILED")
AXIS_ERROR(ERR_SENSORLESS_FAILED,   0x00000080, "SENSORLESS_FAILED")
AXIS_ERROR(ERR_ENCODER_FAILED,      0x00000100, "ENCODER_FAILED")
AXIS_ERROR(ERR_CONTROLLER_FAILED,   0x00000200, "CONTROLLER_FAILED")
AXIS_ERROR(ERR_WATCHDOG_EXPIRED,    0x00000800, "WATCHDOG_EXPIRED")
AXIS_ERROR(ERR_ESTOP_REQUESTED,     0x00004000, "ESTOP_REQUESTED")
