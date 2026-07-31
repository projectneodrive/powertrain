// ============================================================================
//  fb_ov_trip.h — FUNCTION_BLOCK OverVoltageTrip
//
//    VAR_INPUT   vbus            : filtered bus volts
//    VAR_INPUT   already_faulted : a fault is already latched
//    VAR_OUTPUT  (return)        : TRUE on the scan the trip must fire
//
//  Stage 3 of the DC-bus protection, and the last resort — we only reach it if
//  stage 1 (dissipation into the brake resistor) and stage 2 (braking-torque
//  derate) were not enough. The threshold ordering in motor_config.h is what
//  guarantees the resistor always had its chance first.
//
//  The debounce is not optional: a transient, or a run of corrupted samples,
//  must NEVER latch the fault. Two consecutive scans of PRG_SAFETY's bus block
//  (200 Hz) is ~10 ms of sustained over-voltage.
//
//  What tripping costs, on this board: cutting EN_GATE also cuts GVDD, hence
//  the brake (the LM5109B's VDD comes from the DRV8301). Past OV_TRIP there is
//  no dissipation left and the motor is simply freewheeling. That is the
//  intended behaviour — the fault is a last resort, not a regulation mode.
//  TODO: to keep dissipation alive through a fault, hold EN_GATE high and cut
//  the motor via TIM1's BDTR.MOE (six motor gates Hi-Z, GVDD retained).
// ============================================================================
#pragma once
#include "plc/plc_std_fb.h"
#include "config/motor_config.h"

namespace fb {

class OverVoltageTrip {
 public:
  bool operator()(float vbus, bool already_faulted) {
    bool elapsed = _debounce(vbus > CFG_VBUS_OV_TRIP);
    return elapsed && !already_faulted;
  }

 private:
  // 2 scans of the CFG_BUS_SAFETY_HZ block ~= 10 ms.
  plc::TON _debounce{plc::scansFor(10, 1000 / CFG_BUS_SAFETY_HZ)};
};

} // namespace fb
