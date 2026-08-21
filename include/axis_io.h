// ============================================================================
//  axis_io.h  —  The axis command/telemetry block: the part of the process
//  image that the CANSimple fieldbus driver (lib/odrive_can) is mapped onto.
//
//  It lives here, under the project's include/ dir, rather than inside
//  lib/odrive_can, because it is PROCESS DATA, not protocol. The CAN driver
//  reads and writes these variables; the control programs read and write the
//  same variables; neither knows about the other. That is the IEC model for an
//  I/O driver, and it is what lets the CAN layer be swapped or removed without
//  touching the control logic.
//
//  The enums below are the axis vocabulary shared by both sides (states, modes,
//  error bits). The wire encoding itself — arbitration ids, byte packing — stays
//  in lib/odrive_can/odrive_can.h.
//
//  All firmware-side values are SI/rad; the rev/Nm conversions happen in the CAN
//  driver.
// ============================================================================
#pragma once
#include <Arduino.h>

namespace odcan {

// ---- ODrive enums (fw-v0.5.6 values) ---------------------------------------
// Generated from include/axis_vocab.h, which also carries the operator-facing
// name of each value. can_utilities and the web GUI's CAN Devices page decode
// heartbeats and error words from that same table, so a new error bit is named
// everywhere at once instead of showing up as raw hex in two places out of
// three. Add values THERE, not here.
enum AxisState : uint8_t {
#define AXIS_STATE(name, value, label) name = value,
#define AXIS_MODE(name, value, label)
#define AXIS_ERROR(name, value, label)
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
};

enum ControlMode : uint8_t {
#define AXIS_STATE(name, value, label)
#define AXIS_MODE(name, value, label) name = value,
#define AXIS_ERROR(name, value, label)
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
};

enum AxisErrorBits : uint32_t {
  ERR_NONE = 0x00000000,   // the absence of bits; not in the table, see there
#define AXIS_STATE(name, value, label)
#define AXIS_MODE(name, value, label)
#define AXIS_ERROR(name, value, label) name = value,
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
};

// ---- Shared command + telemetry block --------------------------------------
struct AxisIO {
  // commands (fieldbus/console write, control programs read)
  volatile bool     armed            = false;   // CLOSED_LOOP requested
  volatile bool     estop            = false;   // latched
  volatile uint8_t  control_mode     = CTRL_TORQUE;
  volatile uint8_t  input_mode       = 1;       // PASSTHROUGH
  volatile float    input_pos        = 0.0f;    // rad
  volatile float    input_vel        = 0.0f;    // rad/s
  volatile float    input_torque     = 0.0f;    // Nm
  volatile float    vel_limit        = 0.0f;    // rad/s
  volatile float    current_limit    = 0.0f;    // A
  volatile float    pos_gain         = 0.0f;    // (rad/s)/rad  — P_angle.P (cmd G/PP)
  volatile float    pos_int_gain     = 0.0f;    // P_angle.I    (cmd PI, serial only)
  volatile float    pos_d_gain       = 0.0f;    // P_angle.D    (cmd PD, serial only)
  volatile bool     req_pos_gains    = false;   // pos I/D changed -> apply
  volatile float    vel_gain         = 0.0f;    // Nm/(rad/s)   — velocity PID P
  volatile float    vel_int_gain     = 0.0f;    // Nm/(rad/s)/s — velocity PID I
  volatile float    vel_d_gain       = 0.0f;    // Nm/(rad/s²)  — velocity PID D (serial only)
  volatile bool     req_vel_gains    = false;   // gains changed -> apply
  volatile float    cur_p_gain       = 0.0f;    // V/A          — current PID P (cmd JP)
  volatile float    cur_int_gain     = 0.0f;    // current PID I (cmd JI)
  volatile float    cur_d_gain       = 0.0f;    // current PID D (cmd JD)
  volatile bool     req_cur_gains    = false;   // current gains changed -> apply
  volatile uint32_t last_setpoint_ms = 0;       // watchdog feed
  volatile bool     req_reboot       = false;
  volatile bool     req_clear_errors = false;
  volatile bool     req_characterise = false;   // measure motor R/L (MOTOR_CALIBRATION)
  volatile bool     new_mode         = false;   // control_mode changed by CAN

  // telemetry (control programs write, fieldbus/console read)
  volatile float    pos_rev     = 0.0f;         // rev
  volatile float    vel_rev     = 0.0f;         // rev/s
  volatile float    iq_setpoint = 0.0f;         // A
  volatile float    iq_measured = 0.0f;         // A
  volatile float    vbus        = 0.0f;         // V
  volatile float    ibus        = 0.0f;         // A
  volatile uint32_t axis_error  = 0;            // ORed AxisErrorBits
  volatile uint8_t  cur_state   = AXIS_IDLE;    // current AxisState

  // Reserved, always 0. This firmware reports every condition through
  // axis_error, for which axis_vocab.h defines names all three projects share.
  // These exist so the ODrive Get_*_Error frames are well-formed and decode as
  // "no further detail". Populating them needs a sub-error vocabulary in
  // axis_vocab.h first, which is a three-repo table change, not a firmware one —
  // inventing values here would put magic numbers in three places that disagree.
  volatile uint32_t motor_error = 0;
  volatile uint32_t encoder_error = 0;
  volatile uint32_t controller_error = 0;

  // ---- axis_error accessors --------------------------------------------------
  // NEVER write axis_error with |=. It is raised from tasks at different
  // priorities (safety preempts comms freely) and, on the CAN station, from a
  // different core. `|=` on a volatile is LDR/ORR/STR: a bit raised by the
  // preempting side between the load and the store is silently lost — and the
  // losing side is always the safety side.
  void raiseError(uint32_t bits) volatile {
    __atomic_fetch_or(&axis_error, bits, __ATOMIC_RELAXED);
  }

  // Clears only the bits the caller OBSERVED, so a fault raised after the
  // caller's snapshot survives the clear.
  void clearErrorBits(uint32_t mask) volatile {
    __atomic_fetch_and(&axis_error, ~mask, __ATOMIC_RELAXED);
  }
};

} // namespace odcan
