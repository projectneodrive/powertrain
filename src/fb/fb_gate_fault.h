// ============================================================================
//  fb_gate_fault.h — FUNCTION_BLOCK GateFault
//
//    VAR_INPUT   n_fault_asserted : raw nFAULT read (active low, already inverted)
//    VAR_INPUT   stage_enabled    : EN_GATE is high
//    VAR_OUTPUT  (return)         : TRUE on the scan the emergency cut must fire
//
//  nFAULT is only meaningful while the driver is supposed to be running; with
//  EN_GATE low the line is not driven and must not be interpreted.
//
//  Sampled at 1 kHz — full rate, unlike the bus voltage — because unlike Vbus
//  this is a digital read costing nothing, and it is the fast fault detection.
//  The debounce is 11 consecutive scans; that odd-looking number is the
//  original behaviour preserved exactly (a `> 10` counter test), and at 1 kHz
//  it is ~11 ms.
// ============================================================================
#pragma once
#include "plc/plc_std_fb.h"

namespace fb {

class GateFault {
 public:
  bool operator()(bool n_fault_asserted, bool stage_enabled) {
    return _debounce(stage_enabled && n_fault_asserted);
  }

 private:
  plc::TON _debounce{11};   // 11 consecutive 1 ms scans
};

} // namespace fb
