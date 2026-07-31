// ============================================================================
//  fb_regen_derate.h — FUNCTION_BLOCK RegenDerate
//
//    VAR_INPUT   vbus          : filtered bus volts
//    VAR_INPUT   current_limit : the axis current limit currently in force
//    VAR_OUTPUT  (return)      : max |Iq| allowed to OPPOSE rotation
//
//  Stage 2 of the DC-bus protection: linearly withdraw the permission to brake
//  electrically as the bus climbs from REGEN_START to REGEN_FULL.
//
//  A safety net only. The REGEN_* thresholds sit ABOVE the chopper's, so we
//  only sacrifice braking torque once dissipation has already failed to hold
//  the bus.
//
//  Stateless — it is a FUNCTION in IEC terms — but it is kept in fb/ next to
//  the two blocks it forms a protection ladder with, since the three are only
//  correct as a set (see the threshold ordering in motor_config.h).
// ============================================================================
#pragma once
#include "plc/plc_std_fb.h"
#include "config/motor_config.h"

namespace fb {

inline float regenDerate(float vbus, float current_limit) {
  float s = 1.0f - (vbus - CFG_VBUS_REGEN_START)
                 / (CFG_VBUS_REGEN_FULL - CFG_VBUS_REGEN_START);
  return current_limit * plc::LIMIT(0.0f, s, 1.0f);
}

} // namespace fb
