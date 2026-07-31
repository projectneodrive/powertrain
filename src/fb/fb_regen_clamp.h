// ============================================================================
//  fb_regen_clamp.h — FUNCTION_BLOCK RegenClamp
//
//    VAR_IN_OUT  current_sp : the motor's current setpoint, clamped in place
//    VAR_INPUT   velocity   : measured shaft velocity
//    VAR_INPUT   limit      : max |setpoint| allowed to oppose rotation
//
//  Applies the bus regen derate to the torque command, and ONLY to the part of
//  it that opposes rotation (sp * vel < 0, i.e. braking). Motoring torque is
//  never reduced: it does not charge the bus, so limiting it would cost
//  performance for nothing.
//
//  Called after move(), every FOC tick. That placement matters: current_sp
//  persists between two move() calls (MOTION_DOWNSAMPLE = 20), so clamping
//  after move() covers each of the following loopFOC() calls too.
//
//  In the voltage-torque fallback current_sp is in volts rather than amps. The
//  clamp stays homogeneous because current_limit bounds volts there as well.
// ============================================================================
#pragma once
#include "plc/plc_std_fb.h"

namespace fb {

inline void regenClamp(float& current_sp, float velocity, float limit) {
  if (current_sp * velocity < 0.0f)
    current_sp = plc::LIMIT(-limit, current_sp, limit);
}

} // namespace fb
