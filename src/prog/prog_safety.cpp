// ============================================================================
//  prog_safety.cpp — see prog_safety.h.
//
//  THREE RATES IN ONE PROGRAM, and each one is deliberate:
//
//   * nFAULT, 1 kHz (every scan). A digital read costs nothing and this is the
//     fast fault detection.
//
//   * The brake chopper, 1 kHz. Not because the REGULATION needs it — Vbus is
//     only refreshed at 200 Hz — but because the CUT does. A disarm or a fault
//     takes effect in 1 ms instead of 5, for the price of two register writes.
//
//   * Vbus + the protection ladder, CFG_BUS_SAFETY_HZ (200 Hz). This one makes
//     a BLOCKING ADC conversion (~24 us). PRG_SAFETY runs above PRG_FOC, so
//     doing it every 1 ms steals ~24 us from a 50 us FOC budget, at a fixed
//     rate — and RTOS notifications are not queued, so a stolen tick is lost,
//     not caught up. Measured on the bench: a ~5 Hz velocity oscillation
//     appeared the moment regenerative braking was added at 1 kHz. Nothing on a
//     battery bus has 1 kHz dynamics; 200 Hz is plenty.
// ============================================================================
#include "prog/prog_safety.h"
#include "io/io_vbus.h"
#include "io/io_gate.h"
#include "io/io_brake.h"
#include "io/io_motor.h"
#include "fb/fb_regen_derate.h"
#include "gvl/gvl.h"
#include "config/motor_config.h"

namespace prog {

PrgSafety prgSafety;

// ---------------------------------------------------------------------------
void PrgSafety::runBusProtection() {
  float vb = _fbVbus(io::vbus::readRaw());
  gvl::IN.vbus_filt = vb;
  if (vb <= 0.0f) { io::brake::off(); return; }    // no valid measurement yet

  // Stage 3: latched over-voltage fault.
  if (_fbOvTrip(vb, gvl::M.fault)) {
    io::gate::disable();
    gvl::M.fault = true;
    gvl::AXIS.axis_error |= odcan::ERR_DC_BUS_OVER_VOLTAGE;
    Serial.print("[FAULT] DC bus over-voltage: ");
    Serial.print(vb, 1); Serial.println(" V");
  }

  // Stage 2: derate the permitted braking current (consumed by PRG_FOC).
  gvl::Q.regen_iq_limit = fb::regenDerate(vb, io::motor::motor.current_limit);

  // Stage 1 (the chopper) is driven from scan(), at 1 kHz — see the file header.
}

// ---------------------------------------------------------------------------
void PrgSafety::scan() {
  if (++_bus_div >= CFG_BUS_SAFETY_DIV) {
    _bus_div = 0;
    runBusProtection();
  }

  // nFAULT is only meaningful while the driver is supposed to be live.
  bool en_gate = io::gate::enabled();
  if (_fbGateFault(io::gate::faultAsserted(), en_gate)) {
    io::gate::disable();                       // immediate emergency cut
    gvl::M.fault = true;
    gvl::AXIS.axis_error |= odcan::ERR_MOTOR_FAILED;
    en_gate = false;
  }

  // The brake is only physically powered while EN_GATE is high (GVDD comes from
  // the DRV8301): this condition is not a precaution, it is the wiring.
  bool stage_active = gvl::M.foc_ready && !gvl::M.fault && en_gate;
  io::brake::setDuty(_fbChopper(gvl::IN.vbus_filt, stage_active));
}

} // namespace prog
