// ============================================================================
//  safety.cpp — the DC-bus protection ladder, the nFAULT latch and the brake
//  chopper. Runs at 1 kHz, at the top priority. See app.h.
//
//  THREE RATES IN ONE MODULE, each deliberate:
//
//   * nFAULT, 1 kHz (every call). A digital read costs nothing and this is the
//     fast fault detection.
//   * The brake chopper, 1 kHz. Not because the REGULATION needs it — Vbus is
//     only refreshed at 200 Hz — but because the CUT does. A disarm or a fault
//     takes effect in 1 ms instead of 5, for two register writes.
//   * Vbus + the protection ladder, CFG_BUS_SAFETY_HZ (200 Hz). This one makes
//     a BLOCKING ADC conversion (~24 us). This module runs above FOC, so doing
//     it every 1 ms steals ~24 us from a 50 us FOC budget at a fixed rate — and
//     RTOS notifications are not queued, so a stolen tick is lost, not caught
//     up. Measured on the bench: a ~5 Hz velocity oscillation appeared the
//     moment regenerative braking was added at 1 kHz. Nothing on a battery bus
//     has 1 kHz dynamics; 200 Hz is plenty.
//
//  THE LADDER, softest to hardest. The threshold ordering in motor_config.h is
//  what guarantees the resistor always gets its chance before braking torque is
//  sacrificed, and before the fault:
//
//     1. chopper      dissipate into the AUX resistor      (removes energy)
//     2. regen derate withdraw permission to brake         (adds none)
//     3. OV trip      latched fault, gates off             (last resort)
//
//  /!\ NOTHING IN THIS FILE MAY PRINT. This module runs at PRIO_SAFETY, one
//  above the FOC loop, and the Arduino core busy-waits when the serial TX
//  buffer is full - so a print here starves commutation for milliseconds. Set a
//  say_* flag in state::safety instead and let app::console emit it.
// ============================================================================
#include "app.h"

#include <Arduino.h>

#include "config/motor_config.h"
#include "config/tasks_config.h"
#include "io/io.h"
#include "io/io_motor.h"
#include "state.h"
#include "util/timers.h"

namespace app {
namespace safety {
namespace {

// ---- Vbus conditioning -------------------------------------------------------
// Median-of-3 then a first-order low-pass, in that order. The median is not
// cosmetic: it rejects a single wild sample — a conversion aborted by an
// injected trigger, a commutation transient — BEFORE it can enter the filter
// state, where a low-pass would smear it across the next several samples.
// An invalid sample HOLDS the output rather than resetting it: losing one
// conversion should not make the bus voltage momentarily unknown.
float s_prev0 = -1.0f, s_prev1 = -1.0f;   // the two previous RAW samples
float s_vbus  = 0.0f;

float median3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

float filterVbus(float raw_pin_v) {
  if (raw_pin_v >= 0.0f) {
    float v = raw_pin_v * CFG_VBUS_DIV;
    if (s_prev0 < 0.0f) { s_prev0 = v; s_prev1 = v; }   // prime
    float m = median3(s_prev0, s_prev1, v);
    s_prev0 = s_prev1; s_prev1 = v;
    // NOTE: a fixed alpha is a SAMPLE-count filter, not a time-constant one. At
    // the 200 Hz sample rate this is tau ~= 12 ms. It was commissioned on the
    // bench and it feeds both the chopper and the OV trip — retuning
    // CFG_BUS_SAFETY_HZ changes this time constant with it.
    s_vbus = (s_vbus <= 0.0f) ? m : s_vbus + 0.33f * (m - s_vbus);
  }
  return s_vbus;
}

// ---- Stage 3: latched over-voltage ------------------------------------------
// What tripping costs on this board: cutting EN_GATE also cuts GVDD, hence the
// brake (the LM5109B's VDD comes from the DRV8301). Past OV_TRIP there is no
// dissipation left and the motor simply freewheels. That is intended — the
// fault is a last resort, not a regulation mode.
// TODO: to keep dissipation alive through a fault, hold EN_GATE high and cut
// the motor via TIM1's BDTR.MOE (six motor gates Hi-Z, GVDD retained).
//
// The debounce is not optional: a transient, or a run of corrupted samples,
// must NEVER latch the fault. At 200 Hz the preset is two samples = 5 ms of
// SUSTAINED over-voltage, and up to 10 ms of worst-case latency from the
// crossing. Those are different numbers and worth keeping apart: the sustained
// window rejects a transient, the latency is what the bus capacitance survives.
util::Debounce s_ov_debounce{10, 1000 / CFG_BUS_SAFETY_HZ};

// Lowering CFG_BUS_SAFETY_HZ silently erodes that debounce, and at 100 Hz it
// would vanish: callsFor(10, 10) == 1, i.e. the last-resort fault would latch on
// a SINGLE ADC sample. motor_config.h only requires the rate to divide 1000, so
// nothing else stops that edit.
static_assert(util::callsFor(10, 1000 / CFG_BUS_SAFETY_HZ) >= 2,
              "CFG_BUS_SAFETY_HZ is too low to debounce the OV trip over more "
              "than one sample - raise the rate or raise the preset here.");

// ---- The ladder itself ------------------------------------------------------
// The ordering below is declared MANDATORY in motor_config.h and in this file's
// header comment, and until the pack-voltage table landed nothing checked it.
// Now that one #define picks between six threshold columns, a mis-edited column
// must fail the BUILD, not the bench.
static_assert(CFG_BRAKE_VBUS_OFF   < CFG_BRAKE_VBUS_ON
           && CFG_BRAKE_VBUS_ON    < CFG_VBUS_REGEN_START
           && CFG_VBUS_REGEN_START < CFG_VBUS_REGEN_FULL
           && CFG_VBUS_REGEN_FULL  < CFG_VBUS_OV_TRIP,
              "DC-bus ladder out of order for this CFG_PACK_VOLTAGE/CFG_BUS_SOURCE "
              "combination - the resistor must get its chance before braking torque "
              "is cut, and both before the fault. Fix the column in motor_config.h.");

// The trip is compared against a reading scaled by CFG_VBUS_DIV from a 3.3 V
// ADC, so above full scale the measurement pins BELOW the threshold and the
// last-resort fault silently becomes unreachable.
static_assert(CFG_VBUS_OV_TRIP < 3.3f * CFG_VBUS_DIV,
              "CFG_VBUS_OV_TRIP is above the Vbus ADC full scale (3.3 V * "
              "CFG_VBUS_DIV) - the trip could never fire because the reading "
              "saturates first. Lower the trip or fit a different divider.");

static_assert(CFG_BRAKE_VBUS_ON > CFG_VBUS_NOMINAL,
              "CFG_BRAKE_VBUS_ON is at or below the normal working voltage: the "
              "chopper would modulate continuously in ordinary use.");

static_assert(CFG_VOLT_LIMIT <= CFG_VBUS_NOMINAL,
              "CFG_VOLT_LIMIT exceeds the bus it is drawn from.");

// P_peak = duty * Vbus^2 / R. The duty ceiling is per-column precisely so this
// stays near constant as the pack voltage rises; see the derivations in
// motor_config.h.
static_assert(CFG_BRAKE_P_PEAK_W <= CFG_BRAKE_P_MAX_W,
              "CFG_BRAKE_MAX_DUTY would exceed the AUX resistor's rating at this "
              "pack voltage - lower the duty for that column, or raise "
              "CFG_BRAKE_P_MAX_W to match the resistor actually fitted.");

static_assert(CFG_BRAKE_P_CONT <= CFG_BRAKE_P_MAX_W,
              "the AUX resistor cannot take more CONTINUOUSLY than it takes at "
              "its peak - one of the two ratings is wrong.");

// ---- The counter-torque brake's operating window -----------------------------
// Same principle as the ladder above: a threshold set where the mode could
// never engage, or could only engage somewhere the inverter does not control
// the current, must fail the BUILD rather than be discovered on the rig.
static_assert(CFG_GEAR_RATIO > 0.0f,
              "CFG_GEAR_RATIO must be positive - it is a motor/crank speed "
              "ratio, and the telemetry is divided through it.");

static_assert(CFG_REGEN_OMEGA_OFF < CFG_REGEN_OMEGA_ON,
              "regen speed hysteresis is inverted: OFF must be below ON, or "
              "the latch can never release.");

// The braking current's sign comes from copysignf(omega), and at omega exactly
// +/-0 that sign is meaningless while BOTH guards in app::foc are no-ops -
// x * 0 is neither > 0 nor < 0. A zero threshold would therefore be the one
// speed at which nothing protects the axis, so neither end may be zero.
static_assert(CFG_REGEN_OMEGA_FLOOR > 0.0f && CFG_REGEN_OMEGA_ON > 0.0f,
              "CFG_REGEN_OMEGA_FLOOR and CFG_REGEN_OMEGA_ON must both be "
              "strictly positive: the brake must never evaluate its sign at "
              "omega == 0, where app::foc's sign check and its regen clamp are "
              "both no-ops. Check CFG_REGEN_EDGES_MIN too - it scales the "
              "floor.");

// The one that was missing, and the reason the brake once oscillated instead of
// braking. The command's SIGN is the sign of omega, so engaging where the
// velocity estimate cannot resolve its own sign produces a current command that
// flips with the noise - it motors the shaft in alternating directions and
// collapses the bus. Constant torque made that quieter than the constant-power
// law did (a fixed magnitude rather than a saturated one) but not safer.
// CFG_REGEN_OMEGA_FLOOR is derived from the fitted sensor; see motor_config.h.
static_assert(CFG_REGEN_OMEGA_ON >= CFG_REGEN_OMEGA_FLOOR,
              "CFG_REGEN_OMEGA_ON is inside the velocity estimate's noise "
              "floor, where the sign of omega - and therefore the sign of the "
              "braking current - is quantisation rather than measurement. "
              "Raise it to at least CFG_REGEN_OMEGA_FLOOR (printed by the boot "
              "banner and the 'Q' line). Note these thresholds are MOTOR rad/s "
              "and deliberately do NOT scale with CFG_GEAR_RATIO: use "
              "CFG_REGEN_CRANK_RPM() to set them from a cadence.");

// A ramp narrower than the hysteresis band would step the torque in at the full
// value the instant the latch closes, which the shaft feels as a grab.
static_assert(CFG_REGEN_RAMP_BAND > 0.0f,
              "CFG_REGEN_RAMP_BAND must be positive - the law divides by it.");

// The hard one. Past CFG_BEMF_OMEGA_MAX the motor's own back-EMF exceeds the
// bus and the free-wheel diodes rectify into it regardless of what the gates
// do. Engaging a braking law there would command a current the bridge cannot
// source, on top of a bus already being force-charged.
static_assert(CFG_REGEN_OMEGA_ON < CFG_BEMF_OMEGA_MAX,
              "CFG_REGEN_OMEGA_ON is at or above the speed where the line "
              "back-EMF reaches CFG_VOLT_LIMIT - the inverter does not control "
              "the current there. Lower it, or fit a motor with a higher KV.");

// Both ends of the window at once: the floor must leave room under the ceiling,
// or there is no speed at which this mode can legally run.
static_assert(CFG_REGEN_OMEGA_FLOOR < CFG_BEMF_OMEGA_MAX,
              "the velocity-estimate floor is above the back-EMF ceiling: this "
              "motor has no speed range where the brake is both measurable and "
              "controllable. A lower pole count or a higher KV opens the "
              "window.");

static_assert(CFG_REGEN_OMEGA_ON < CFG_VEL_LIMIT_MAX,
              "the regen engagement threshold is outside the reachable speed "
              "range: CTRL_BRAKE could never activate.");

// brakeCurrent() clamps the setpoint against the LIVE current limit, so this
// relationship holds today and the final clamp in that function is unreachable.
// If the two are ever decoupled, that clamp would start absorbing the
// difference silently - with nothing in the telemetry to show it.
static_assert(CFG_REGEN_T_MAX <= CFG_REGEN_I_MAX * CFG_KT,
              "CFG_REGEN_T_MAX exceeds the torque CFG_REGEN_I_MAX can deliver; "
              "the clamp at the end of brakeCurrent() would silently eat the "
              "difference.");

static_assert(CFG_REGEN_T_SLEW >= 0.0f,
              "CFG_REGEN_T_SLEW is a rate in Nm/s; 0 disables the ramp and "
              "negative is meaningless.");

// FOC updates per ELECTRICAL revolution at the top of the speed range. Below
// ~15 the commutation angle is refreshed too coarsely for the current loop to
// hold a clean dq frame. f_elec = pp * omega / (2*pi), so the count is
// FOC_TICK_HZ * 2*pi / (pp * omega); the form below avoids the division.
// This is what a high-pole-count motor runs into first at G = 72.
static_assert((float)FOC_TICK_HZ * TWO_PI
                  >= 15.0f * (float)CFG_POLE_PAIRS * CFG_VEL_LIMIT_MAX,
              "fewer than 15 FOC updates per electrical revolution at "
              "CFG_VEL_LIMIT_MAX - lower CFG_POLE_PAIRS, lower the velocity "
              "ceiling, or raise FOC_TICK_HZ.");

// ---- Stage 2: regen derate ---------------------------------------------------
// Linearly withdraw permission to brake electrically as the bus climbs from
// REGEN_START to REGEN_FULL. A safety net only: those thresholds sit ABOVE the
// chopper's, so torque is only sacrificed once dissipation has already failed.
float regenDerate(float vbus, float current_limit) {
  float s = 1.0f - (vbus - CFG_VBUS_REGEN_START)
                 / (CFG_VBUS_REGEN_FULL - CFG_VBUS_REGEN_START);
  return current_limit * util::limit(0.0f, s, 1.0f);
}

// ---- Resistor thermal model ---------------------------------------------------
// Stage 2 above answers "is the bus climbing?". This answers a different
// question the bus voltage cannot: "has the resistor been taking more than it
// can shed?". On a PSU bus they diverge badly — the chopper holds Vbus flat
// while burning 200 W continuously, so every voltage-based test says everything
// is fine right up until the resistor fails.
//
// A single-pole model on the NORMALISED overload, which is all the accuracy the
// two constants available justify: s_brake_load is dissipation expressed in
// units of the continuous rating, low-passed over the thermal time constant.
// 1.0 means "sitting exactly at the continuous rating". It is driven by the
// duty actually applied, so on a battery bus — where the chopper mostly idles —
// it decays to zero and the derate is inert without needing a #if.
//
// Deliberately NOT a temperature: calling it degrees would invite tuning it
// against a thermocouple reading, and the model has no thermal resistance term
// to make that meaningful.
float s_brake_load = 0.0f;

float brakeThermalDerate(float p_res) {
  const float dt = 1.0f / (float)CFG_BUS_SAFETY_HZ;
  s_brake_load += (p_res / CFG_BRAKE_P_CONT - s_brake_load)
                  * dt / CFG_BRAKE_THERMAL_TAU_S;
  // Derate linearly from DERATE_START to 1.0 of the rating rather than cutting
  // at the limit: a step here would be felt by the rider as the brake letting
  // go, and would set up a slow hunt around the threshold.
  float over = (s_brake_load - CFG_BRAKE_DERATE_START)
               / (1.0f - CFG_BRAKE_DERATE_START);
  return util::limit(0.0f, 1.0f - over, 1.0f);
}

// ---- Stage 1: the brake chopper ----------------------------------------------
// Engage above VBUS_ON, release only on falling back below VBUS_OFF. Once
// engaged the duty is computed from VBUS_OFF, NOT from VBUS_ON — otherwise it
// would be negative (hence zero) across the whole hysteresis band and the
// chopper would chatter around VBUS_ON instead of holding the bus inside it.
util::Hysteresis s_chopper;

float chopperDuty(float vbus, bool stage_active) {
  // Safe default: stage disarmed/faulted, or an invalid reading -> OFF, and
  // drop the latch so re-arming starts clean.
  if (!stage_active || vbus <= 0.0f) { s_chopper.reset(); return 0.0f; }
  if (!s_chopper(vbus, CFG_BRAKE_VBUS_OFF, CFG_BRAKE_VBUS_ON)) return 0.0f;
  float d = (vbus - CFG_BRAKE_VBUS_OFF) * CFG_BRAKE_GAIN;
  return util::limit(0.0f, d, (float)CFG_BRAKE_MAX_DUTY);
}

// ---- nFAULT ------------------------------------------------------------------
util::Debounce s_gate_debounce{11, SCAN_MS_SAFETY};

uint8_t s_bus_div = 0;

void runBusProtection() {
  float vb = filterVbus(io::vbus::readRaw());
  state::safety.vbus_filt = vb;
  if (vb <= 0.0f) { io::brake::off(); return; }   // no valid measurement yet

  if (s_ov_debounce(vb > CFG_VBUS_OV_TRIP) && !state::safety.fault) {
    io::gate::disable();
    state::safety.fault = true;
    state::axis.raiseError(odcan::ERR_DC_BUS_OVER_VOLTAGE);
    // Announce via app::console, never from here - see state.h FromSafety.
    state::safety.say_ov_vbus = vb;
    state::safety.say_ov_trip = true;
  }

  // Dissipation is computed from the duty that was actually applied on the
  // previous scan. Ibrk in the telemetry is the same quantity as a current, and
  // carries the same caveat: there is no shunt on the AUX branch, so this is
  // what was COMMANDED. A resistor that is not conducting reads as hot here.
  float p_res = io::brake::duty() * vb * vb / CFG_BRAKE_R;
  state::safety.p_res        = p_res;
  state::safety.brake_derate = brakeThermalDerate(p_res);

  // Both derates land on the same knob, and the tighter one wins. They guard
  // different failures — the bus climbing (voltage) and the resistor saturating
  // (energy) — and either alone leaves the other open. Applied downstream at
  // 20 kHz by app::foc, and only to torque that OPPOSES rotation, so motoring
  // is never penalised for the resistor's state.
  float lim = regenDerate(vb, io::motor::motor.current_limit);
  float thermal = io::motor::motor.current_limit * state::safety.brake_derate;
  state::safety.regen_iq_limit = fminf(lim, thermal);
}

}  // namespace

void update() {
  // Clear-errors is consumed HERE, not by control, so that this module is the
  // only writer of the fault latch. Otherwise a clear serviced in another task
  // can erase a fault latched microseconds earlier and re-arm into a live
  // over-voltage — the console runs at 100 ms and comms at 1 ms, so the clear
  // is always at least one safety scan behind whatever it is clearing.
  //
  // There is no window: a still-true condition re-latches further down this
  // same call, microseconds later. And clearing only the bits we SAW means a
  // fault raised by another task after the snapshot survives.
  if (state::axis.req_clear_errors) {
    state::axis.req_clear_errors = false;
    uint32_t seen = state::axis.axis_error;
    state::safety.fault = false;
    state::axis.clearErrorBits(seen);
    state::axis.estop = false;
    state::safety.say_cleared = true;   // printed by app::console, not here
  }

  if (++s_bus_div >= CFG_BUS_SAFETY_DIV) {
    s_bus_div = 0;
    runBusProtection();
  }

  // nFAULT is only meaningful while the driver is supposed to be live.
  bool en_gate = io::gate::enabled();
  if (s_gate_debounce(en_gate && io::gate::faultAsserted())) {
    io::gate::disable();                       // immediate emergency cut
    state::safety.fault = true;
    state::axis.raiseError(odcan::ERR_MOTOR_FAILED);
    en_gate = false;
  }

  // The brake is only physically powered while EN_GATE is high (GVDD comes from
  // the DRV8301): this condition is not a precaution, it is the wiring.
  bool stage_active = state::control.foc_ready && !state::safety.fault && en_gate;
  io::brake::setDuty(chopperDuty(state::safety.vbus_filt, stage_active));
}

}  // namespace safety
}  // namespace app
