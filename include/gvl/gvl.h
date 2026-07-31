// ============================================================================
//  gvl/gvl.h  —  VAR_GLOBAL: the process image.
//
//  Every variable shared between programs lives here and nowhere else. Each one
//  names its SINGLE WRITER; that discipline is not stylistic, it is what makes
//  the lock-free sharing between tasks correct on this MCU:
//
//    A float written by exactly one writer is atomic on Cortex-M4, so any other
//    task can read it without a critical section. Pairs of variables are NOT.
//    That is why pos/vel are published here as two plain floats by PRG_FOC
//    instead of letting other tasks call sensor.getAngle(): getAngle() reads
//    (full_rotations, angle_prev), a non-atomic pair updated at 20 kHz, and
//    reading it from another task returns torn values = ±1 turn spikes in
//    pos_rev.
//
//  IEC areas, mapped onto the three structs below:
//    IN  (%I) — measured/acquired values, produced by the I/O layer or by the
//               program that owns the corresponding device
//    Q   (%Q) — commands produced by the control logic and consumed downstream
//    M   (%M) — internal machine state (retained across scans, not I/O)
//    AXIS     — the fieldbus-mapped command/telemetry block (see gvl/axis_io.h)
// ============================================================================
#pragma once
#include <Arduino.h>
#include "gvl/axis_io.h"
#include "config/motor_config.h"

namespace gvl {

// ---- %I : inputs -----------------------------------------------------------
struct Inputs {
  // Sensor telemetry. WRITER: PRG_FOC only (at 1 kHz, every 20th FOC tick).
  // Sensor convention (not yet sign-corrected for sensor_direction).
  volatile float shaft_angle = 0.0f;   // rad, multi-turn
  volatile float shaft_vel   = 0.0f;   // rad/s

  // DC bus. WRITER: PRG_SAFETY only (median-of-3 + LPF, at CFG_BUS_SAFETY_HZ).
  volatile float vbus_filt   = 0.0f;   // V

  // Set once during boot, read-only afterwards: did the low-side current sense
  // come up? False = voltage-torque fallback, which changes the units of every
  // gain and setpoint downstream.
  volatile bool  isense_ok   = false;
};

// ---- %Q : outputs ----------------------------------------------------------
struct Outputs {
  // Motion setpoint handed to motor.move(). Units follow control_mode:
  // rad/s (velocity), rad (position), or A / V (torque, depending on isense_ok).
  // WRITER: PRG_CONTROL only.
  volatile float active_target   = 0.0f;

  // Max |Iq| allowed to OPPOSE rotation (braking), from the bus regen derate.
  // WRITER: PRG_SAFETY. READER: PRG_FOC.
  volatile float regen_iq_limit  = CFG_CURRENT_LIMIT;
};

// ---- %M : internal machine state -------------------------------------------
struct Memory {
  volatile bool fault        = false;  // latched hardware/bus fault
  volatile bool foc_ready    = false;  // closed loop running
  volatile bool calibrated   = false;  // initFOC has succeeded once
  volatile bool req_hall_cal = false;  // serial 'H' -> run the hall angle sequence
};

extern Inputs       IN;
extern Outputs      Q;
extern Memory       M;
extern odcan::AxisIO AXIS;

} // namespace gvl
