// ============================================================================
//  bridge_state.h — everything the control station knows about the axis.
//
//  Split in two on purpose:
//
//    Measured   what came back from the board. Only the CAN RX decoders write
//               it. If a field is stale, its have_* flag says so.
//    Commanded  what we last TOLD the board. The console and the joystick
//               write it; nothing here is a measurement.
//
//  Keeping them apart is what stops the telemetry line from reporting a
//  setpoint as though it were a reading — the two agree only when the axis is
//  tracking, and the interesting moments are exactly the ones where they don't.
//
//  The commanded defaults are the FIRMWARE's defaults (CFG_* from
//  config/motor_config.h), not numbers chosen here. Before that link existed
//  this station armed the motor with a 15 A current limit while the firmware
//  was configured for 4 A.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_config.h"
#include "axis_io.h"   // firmware-shared: AxisState / ControlMode / error bits

namespace bridge {

// ODrive input modes. Only PASSTHROUGH is used: the board does its own setpoint
// ramping (CFG_VEL_ACCEL), so asking the CAN layer to ramp too would fight it.
enum InputMode : uint8_t {
  INPUT_PASSTHROUGH = 1,
};

// ---- what the board told us -----------------------------------------------
struct Measured {
  uint32_t axis_error        = 0;
  uint8_t  heartbeat_state   = odcan::AXIS_IDLE;
  uint32_t last_heartbeat_ms = 0;

  float pos_rad       = 0.0f;
  float vel_rad_s     = 0.0f;
  float iq_setpoint_a = 0.0f;
  float iq_measured_a = 0.0f;
  float vbus_v        = 0.0f;
  float ibus_a        = 0.0f;

  // Answers to explicit requests (serial 'F'), not cyclic.
  uint32_t motor_error      = 0;
  uint32_t encoder_error    = 0;
  uint32_t controller_error = 0;

  bool have_encoder = false;
  bool have_iq      = false;
  bool have_vbus    = false;
};

// ---- what we told the board ------------------------------------------------
struct Commanded {
  uint8_t control_mode = odcan::CTRL_TORQUE;
  uint8_t input_mode   = INPUT_PASSTHROUGH;

  // The ACTIVE setpoint, in the unit of control_mode — this is what the
  // telemetry line reports as `tgt`, mirroring gvl::Q.active_target.
  float target = 0.0f;

  // And one per mode, kept separately because the console acknowledges
  // "old -> new" and those two numbers must be the same quantity. Sharing one
  // field makes a T command after a V command report "torque 5.00 -> 0.25 Nm",
  // where the 5.00 was rad/s. The firmware keeps input_pos/vel/torque apart for
  // the same reason.
  float input_torque = 0.0f;   // Nm
  float input_vel    = 0.0f;   // rad/s
  float input_pos    = 0.0f;   // rad

  // Set_Limits carries both at once, so both must be remembered to change one.
  float vel_limit     = CFG_VEL_LIMIT;
  float current_limit = CFG_CURRENT_LIMIT;

  // Set_Vel_Gains carries P and I together, same reason. CANSimple has no D
  // term — the board's D stays reachable only over its own USB console.
  float vel_gain     = CFG_VEL_P;
  float vel_int_gain = CFG_VEL_I;
  float pos_gain     = CFG_POS_P;
};

struct State {
  Measured  m;
  Commanded c;
  uint32_t  sample_index = 0;
  bool      link_up      = false;   // a heartbeat from the target has been seen

  // The one-shot link-loss safety stop has fired and not yet been re-armed.
  // Surfaced on the `can ...` line so the GUI can say WHY the axis is idle —
  // otherwise a disarmed motor and a motor nobody armed look identical.
  bool      safety_stopped = false;

  // Worst scan duration since the last status line. The number that says
  // whether the loop is keeping up — a healthy scan is microseconds, and
  // anything approaching the heartbeat period means the link checks are
  // sampling too coarsely to be trusted. Consumed by the `can ...` line.
  uint32_t  scan_max_ms = 0;
};

}  // namespace bridge
