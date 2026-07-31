// ============================================================================
//  fb_vel_ramp.h — FUNCTION velocityRamp
//
//    VAR_INPUT   current  : the setpoint currently in force (rad/s)
//    VAR_INPUT   cmd      : requested velocity (rad/s), already clamped
//    VAR_INPUT   scan_ms  : period of the calling program
//    VAR_OUTPUT  (return) : the setpoint after one scan of slewing
//
//  Slews the velocity SETPOINT at CFG_VEL_ACCEL instead of stepping it. This is
//  distinct from CFG_VEL_RAMP, which limits the PID's output CURRENT: this one
//  smooths the target, that one smooths the effort.
//
//  Without it, a step (e.g. V5 -> V10) makes a bench supply dip and then
//  overshoot, and the regen on the way back trips OV. Too fast a ramp is its
//  own problem: stopping dead at the target excites a speed overshoot on
//  arrival (~5 Hz ringing) which regenerates and makes the brake flicker — felt
//  as a stutter.
//
//  Deliberately a FUNCTION, not a FUNCTION_BLOCK, even though plc::RAMP_REAL
//  exists and would fit the shape. The ramp's state IS gvl::Q.active_target,
//  which is process data owned by the process image: it is zeroed on disarm and
//  written directly by the position and torque modes. Duplicating it inside a
//  block would leave the copy stale across those transitions, and a re-arm
//  would resume slewing from a setpoint nobody asked for.
// ============================================================================
#pragma once
#include "plc/plc_std_fb.h"
#include "config/motor_config.h"

namespace fb {

inline float velocityRamp(float current, float cmd, uint32_t scan_ms) {
  if (CFG_VEL_ACCEL <= 0.0f) return cmd;              // 0 = direct step
  float step = CFG_VEL_ACCEL * ((float)scan_ms * 0.001f);
  return current + plc::LIMIT(-step, cmd - current, step);
}

} // namespace fb
