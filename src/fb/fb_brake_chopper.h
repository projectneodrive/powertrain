// ============================================================================
//  fb_brake_chopper.h — FUNCTION_BLOCK BrakeChopper
//
//    VAR_INPUT   vbus         : filtered bus volts. <= 0 => invalid => OFF
//    VAR_INPUT   stage_active : power stage genuinely live (closed loop, no
//                               fault, EN_GATE high). FALSE => OFF, no condition
//    VAR_OUTPUT  (return)     : duty to apply, 0 = stop the bridge
//
//  Stage 1 of the DC-bus protection, and the only one that actually removes
//  energy from the bus instead of refusing to add more.
//
//  HYSTERESIS: engage above VBUS_ON, release only on falling back below
//  VBUS_OFF. Once engaged the duty is computed from VBUS_OFF, NOT from VBUS_ON
//  — otherwise it would be negative (hence zero) across the whole hysteresis
//  band, and the chopper would just chatter around VBUS_ON instead of holding
//  the bus inside the band.
//
//  Pure logic: it decides a duty and returns it. Programming TIM2 is
//  io/io_brake.h's job. Note that the stage_active input is not a software
//  precaution — with EN_GATE low the AUX half-bridge has no GVDD and physically
//  cannot switch (see io_brake.h) — but keeping the registers in a defined
//  state regardless is what makes the re-arm transition predictable.
// ============================================================================
#pragma once
#include "plc/plc_std_fb.h"
#include "config/motor_config.h"

namespace fb {

class BrakeChopper {
 public:
  float operator()(float vbus, bool stage_active) {
    // Safe default: stage disarmed/faulted, or an invalid bus reading -> OFF,
    // and drop the latch so re-arming starts from a clean state.
    if (!stage_active || vbus <= 0.0f) { _hyst.reset(); return 0.0f; }

    if (!_hyst(vbus, CFG_BRAKE_VBUS_OFF, CFG_BRAKE_VBUS_ON)) return 0.0f;

    float d = (vbus - CFG_BRAKE_VBUS_OFF) * CFG_BRAKE_GAIN;
    return plc::LIMIT(0.0f, d, (float)CFG_BRAKE_MAX_DUTY);
  }

  bool engaged() const { return _hyst.Q(); }

 private:
  plc::HYSTERESIS _hyst;
};

} // namespace fb
