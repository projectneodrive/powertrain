// ============================================================================
//  state.h — the values modules share, grouped by WHO WRITES THEM.
//
//  THE RULE: exactly one module writes each struct below. Everyone may read
//  anything. Writing outside your own struct is a bug, and the struct name is
//  there so a reviewer sees it without hunting for a comment.
//
//  Why that is enough, with no mutex anywhere: a 32-bit scalar with a single
//  writer is atomic on Cortex-M4, so any task can read it mid-update and still
//  get a value that was real at some instant.
//
//  PAIRS ARE NOT ATOMIC. If two values only make sense together, the writer
//  publishes the DERIVED value instead. That is why p_elec exists here rather
//  than four dq terms, and why pos/vel are published here rather than read from
//  the sensor: Sensor::getAngle() reads (full_rotations, angle_prev), a pair
//  updated at 20 kHz, and reading it from another task returned torn values —
//  observed as +/-1 turn spikes in pos_rev.
//
//  Everything here is SI/rad. Only the CAN boundary converts to rev/Nm.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "axis_io.h"
#include "config/motor_config.h"

namespace state {

// Writer: app::foc, on its 1 kHz publish divider (the 20 kHz path writes none
// of this — see the divider in foc.cpp).
struct FromFoc {
  volatile float shaft_angle = 0.0f;   // rad, multi-turn, SENSOR convention
  volatile float shaft_vel   = 0.0f;   // rad/s, sensor convention
  volatile float p_elec      = 0.0f;   // W, Vq*Iq + Vd*Id — see the pairs rule
  volatile float iq_measured = 0.0f;   // A
  volatile float iq_setpoint = 0.0f;   // A
};

// Writer: app::safety. Owns the fault latch outright: it both sets it and
// consumes the clear request, so no other task can erase a fault it just saw.
struct FromSafety {
  volatile float vbus_filt      = 0.0f;              // V, median-of-3 + LPF
  volatile float regen_iq_limit = CFG_CURRENT_LIMIT; // A, max |Iq| opposing rotation
  volatile bool  fault          = false;             // latched
};

// Writer: app::control.
struct FromControl {
  // Motion setpoint handed to motor.move(). Units follow the control mode:
  // rad/s, rad, or A / V for torque depending on isense_ok.
  volatile float active_target = 0.0f;
  volatile bool  foc_ready     = false;   // closed loop running
  volatile bool  calibrated    = false;   // initFOC has succeeded once
};

// Written once during boot, read-only afterwards. isense_ok false means the
// voltage-torque fallback, which changes the units of every gain downstream.
struct AtBoot {
  volatile bool isense_ok = false;
};

// The one category the single-writer rule does not cover: set by any module,
// consumed-and-cleared by exactly one. Safe for a bool because both sides only
// ever store, never read-modify-write.
struct Requests {
  volatile bool hall_cal = false;   // set: console   consumed: control
};

// Defined here rather than in a .cpp: `inline` gives one shared definition
// across every translation unit that includes this header, which is all a
// separate state.cpp was ever doing.
inline FromFoc     foc;
inline FromSafety  safety;
inline FromControl control;
inline AtBoot      at_boot;
inline Requests    req;

// The fieldbus-mapped command/telemetry block. Multi-writer BY DESIGN — the CAN
// driver, the console and control all command the axis — which is why it is
// deliberately not one of the structs above. Its one dangerous field has atomic
// accessors; see axis_io.h.
inline odcan::AxisIO axis;

}  // namespace state
