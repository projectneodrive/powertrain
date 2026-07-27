// ============================================================================
//  HallSensorSmoothVel — HallSensor with two fixes for the 26pp hub motor.
//
//  1) VELOCITY (getVelocity): the stock HallSensor derives speed from the single
//     most recent inter-edge interval, assuming each sector is exactly 60 deg
//     elec. Real hall/magnet spacing isn't uniform, so that estimate zigzags
//     edge-to-edge (worse at speed). We use the generic Sensor::getVelocity(),
//     which measures angle over its own elapsed time spanning several edges;
//     CFG_HALL_VEL_WINDOW (min_elapsed_time) widens that window. A stall guard
//     off angle_prev_ts is added back (the generic version has none).
//
//  2) COMMUTATION (sector_offset / offsets_active): HallSensor::update() sets
//     angle_prev to each sector's START angle on the same uniform 60 deg grid,
//     so the FOC commutation angle is off by the per-sector placement error ->
//     torque ripple at every speed. calibrateHallAngles() (main.cpp, 'H') spins
//     open-loop to measure the true per-sector angle and stores 6 mean-zero
//     MECHANICAL corrections here (global offset stays in zero_electric_angle).
//     We add them to angle_prev BEFORE the SmoothingSensor copies/interpolates.
// ============================================================================
#pragma once
#include <SimpleFOC.h>

class HallSensorSmoothVel : public HallSensor {
 public:
  using HallSensor::HallSensor;

  // Per-sector commutation corrections (mechanical rad), indexed by
  // electric_sector 0..5. Zero + inactive until calibrated (see main.cpp 'H').
  float sector_offset[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  bool  offsets_active   = false;

  void update() override {
    HallSensor::update();          // fills angle_prev (uniform 60deg grid), full_rotations
    if (!offsets_active) return;
    int8_t s = electric_sector;    // kept live by the hall ISR + HallSensor::update()
    if (s < 0 || s > 5) return;
    angle_prev += sector_offset[s];
    // Keep the (full_rotations, angle_prev) pair consistent across the 0/2PI
    // seam so getAngle() stays continuous. Corrections are tiny (< a few deg /
    // pole_pairs) so at most one wrap step is ever needed.
    if (angle_prev < 0.0f)        { angle_prev += _2PI; full_rotations -= 1; }
    else if (angle_prev >= _2PI)  { angle_prev -= _2PI; full_rotations += 1; }
  }

  // Kept consistent for any code path that reads getSensorAngle() directly
  // (the main commutation path goes through angle_prev, handled in update()).
  float getSensorAngle() override {
    float a = HallSensor::getSensorAngle();
    if (offsets_active) {
      int8_t s = electric_sector;
      if (s >= 0 && s <= 5) a += sector_offset[s];
    }
    return a;
  }

  float getVelocity() override {
    float v = Sensor::getVelocity();
    // Stall guard: Sensor::getVelocity() has no timeout; if no hall edge ever
    // arrives again it keeps returning the last value. Zero it after 200 ms
    // without a real edge (angle_prev_ts pinned by HallSensor::update()).
    if ((unsigned long)(micros() - (unsigned long)angle_prev_ts) > stall_timeout_us) {
      velocity = 0.0f;
      return 0.0f;
    }
    return v;
  }

  unsigned long stall_timeout_us = 200000; // 200 ms with no hall edge -> assume stopped
};
