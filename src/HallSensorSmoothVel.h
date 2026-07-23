// ============================================================================
//  HallSensorSmoothVel — Hall velocity via multi-edge time/angle averaging.
//
//  Simple FOC's HallSensor::getVelocity() computes speed from the SINGLE most
//  recent inter-edge interval, assuming every hall sector spans exactly
//  2*PI/cpr (60 deg elec). Real hall PCB / magnet ring spacing is never
//  perfectly uniform, so that assumption makes the per-edge velocity estimate
//  alternate high/low sector to sector. Worse, since velocity = fixed_angle /
//  elapsed_time, a FIXED mechanical placement error produces a GROWING
//  velocity error as elapsed_time shrinks at higher speed — measured on bench
//  as saccades that get worse, not better, as speed increases.
//
//  The generic Sensor::getVelocity() (common/base_classes/Sensor.cpp) instead
//  measures angle traveled since its own last successful call, over the real
//  elapsed time — which spans as many hall edges as occurred in that window,
//  averaging out their individual placement error. Raising min_elapsed_time
//  (set on the instance after sensor.init(), see CFG_HALL_VEL_WINDOW in
//  board_config.h) forces that window wide enough to span several edges even
//  at speed, instead of the library default of 100us (effectively per-edge).
//
//  HallSensor::update() still drives angle_prev/angle_prev_ts (from the ISR's
//  pulse_timestamp) at full rate, so full-rotation tracking and commutation
//  angle are untouched -- only the *velocity* computation changes.
// ============================================================================
#pragma once
#include <SimpleFOC.h>

class HallSensorSmoothVel : public HallSensor {
 public:
  using HallSensor::HallSensor;

  float getVelocity() override {
    float v = Sensor::getVelocity();
    // Sensor::getVelocity() has no stall detection: if no new hall edge ever
    // arrives again it just keeps returning the last nonzero value forever.
    // HallSensor::getVelocity() guards against this (2x last interval
    // timeout); replicate that here off angle_prev_ts, which HallSensor::
    // update() keeps pinned to the timestamp of the most recent real edge.
    if ((unsigned long)(micros() - (unsigned long)angle_prev_ts) > stall_timeout_us) {
      velocity = 0.0f;
      return 0.0f;
    }
    return v;
  }

  unsigned long stall_timeout_us = 200000; // 200 ms with no hall edge -> assume stopped
};
