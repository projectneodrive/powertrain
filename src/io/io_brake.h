// ============================================================================
//  io_brake.h — I/O module for the brake-resistor half-bridge (AUX terminals).
//
//  This module owns TIM2/CH3/CH4 and nothing else. TIM1 (motor PWM), TIM3
//  (sensor) and TIM6 (FOC tick) are never reallocated.
//
//  It is PURE OUTPUT: give it a duty, it programs the timer. The decision of
//  WHAT duty to apply — the hysteresis and the proportional law — lives in
//  fb/fb_brake_chopper.h, because that is control logic, not hardware.
//
//  Hardware: DRV8301/ODESC AUX half-bridge — LM5109B gate driver (U7) +
//  NTMFS5C628N (IC15//IC16 high side, IC13//IC14 low side). The resistor is
//  wired between the midpoint (JP2.2 / TP13) and ground (JP2.1 / TP12), so it
//  is the HIGH FET that dissipates. The topology and the mandatory complementary
//  drive are documented in config/hw_pinout.h.
//
//  /!\ CRITICAL HARDWARE DEPENDENCY: the LM5109B's VDD is fed from GVDD, the
//  DRV8301's internal gate regulator, which only exists while EN_GATE is high.
//  The brake is therefore PHYSICALLY INOPERATIVE with the stage disarmed — that
//  is not a software decision, it is the board's wiring. Direct consequence:
//  there is NO over-voltage protection at all while the stage is disarmed (a
//  motor driven mechanically at standstill, for instance).
//  TODO: if permanent protection is needed, hold EN_GATE high at all times and
//  cut the motor another way (TIM1's BDTR.MOE = 0 puts the six motor gates in
//  Hi-Z without touching EN_GATE, hence without losing GVDD).
// ============================================================================
#pragma once
#include <Arduino.h>

namespace io {
namespace brake {

// Drive both AUX gates LOW as plain GPIO. Must run at the very top of boot,
// BEFORE anything switches those pins to an alternate function, so the
// half-bridge never passes through an undefined state at power-up.
void preInit();

// Configure TIM2 center-aligned and leave the half-bridge STOPPED.
// Call from the boot sequence AFTER the DRV8301 has been brought up (GVDD).
void init();

// Apply a duty in [0 .. CFG_BRAKE_MAX_DUTY]. Values <= 0 stop the bridge.
void setDuty(float d);

// Immediate stop of both FETs. Safe to call before init() and from any
// context (two register writes).
void off();

// Duty currently applied, for telemetry.
float duty();

} // namespace brake
} // namespace io
