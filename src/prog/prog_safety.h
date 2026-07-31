// ============================================================================
//  prog_safety.h — PROGRAM PRG_SAFETY. Bound to the highest-priority cyclic
//  task, 1 kHz. Owns the DC bus measurement, the three-stage over-voltage
//  protection ladder and the gate-driver fault latch.
// ============================================================================
#pragma once
#include "plc/plc_program.h"
#include "fb/fb_vbus_filter.h"
#include "fb/fb_ov_trip.h"
#include "fb/fb_gate_fault.h"
#include "fb/fb_brake_chopper.h"

namespace prog {

class PrgSafety : public plc::Program {
 public:
  const char* name() const override { return "PRG_SAFETY"; }
  void scan() override;

 private:
  void runBusProtection();

  fb::VbusFilter      _fbVbus;
  fb::OverVoltageTrip _fbOvTrip;
  fb::GateFault       _fbGateFault;
  fb::BrakeChopper    _fbChopper;
  uint8_t             _bus_div = 0;
};

extern PrgSafety prgSafety;

} // namespace prog
