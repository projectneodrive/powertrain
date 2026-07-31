// ============================================================================
//  HybridSensor — hall at low speed, sensorless flux observer at high speed.
//
//  Above ~5 rad/s the hall's 156-states/rev quantization is the smoothness floor
//  (see HallSensorSmoothVel.h). The MESC/Lemming flux observer estimates the
//  rotor angle from the phase currents + applied voltages (BEMF integration),
//  which is continuous (no quantization) once the BEMF is strong enough — at
//  5 rad/s mech (130 rad/s elec) the BEMF is ~6 V, plenty. This wrapper runs
//  BOTH sensors every FOC tick and blends them over [vel_lo, vel_hi]:
//    - below vel_lo : pure hall (unchanged behavior)
//    - above vel_hi : pure observer
//    - in between   : linear blend
//
//  KEY: the FOC commutation is  sensor_direction*pp*getMechanicalAngle() -
//  zero_electric_angle, with a SINGLE zero_electric_angle calibrated for the
//  HALL. The observer's absolute zero is arbitrary (and its sign can be flipped
//  vs the hall). So while the hall is authoritative we (a) latch the observer's
//  direction by comparing velocities, and (b) slew a constant offset so that
//  (obs_angle*dir + offset) tracks the hall angle. Then the same calibrated
//  zero_electric_angle stays valid through the handoff — no commutation jump.
//
//  SAFETY: a wrong observer angle at speed = mis-commutation (violent motion /
//  overcurrent). We never blend toward the observer unless (1) its direction is
//  latched and (2) its velocity agrees with the hall (obs_dv small). The health
//  metric obs_dv (= v_obs - v_hall) and the blend factor are exported for
//  telemetry so the observer can be VERIFIED on the bench (spin up, watch obs_dv
//  stay near 0 across the speed range) BEFORE enabling the handoff.
// ============================================================================
#pragma once
#include <SimpleFOC.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"

class HybridSensor : public Sensor {
 public:
  HybridSensor(Sensor& hall, FOCMotor& motor) : _obs(motor), _hall(hall) {}

  MXLEMMINGObserverSensor _obs;

  // Configuration (set from board_config in setup()).
  bool  enabled = false;      // false => pure hall, identical to the hall-only build
  float vel_lo  = 5.0f;       // rad/s (abs): below this, pure hall
  float vel_hi  = 7.0f;       // rad/s (abs): above this, pure observer

  // Live diagnostics (telemetry / bench verification).
  float blend   = 0.0f;       // 0 = hall, 1 = observer
  float obs_dv  = 0.0f;       // v_obs - v_hall (rad/s) — health: should stay ~0
  bool  healthy = false;      // observer trusted this cycle

  // Init the observer (the hall is init'd separately in setup, before us).
  void initObserver() { _obs.init(); }

  void update() override {
    _hall.update();
    _obs.update();

    float a_hall = _hall.getAngle();          // multi-turn, with SmoothingSensor interpolation
    float v_hall = _hall.getVelocity();
    float a_obs  = _obs.getAngle();
    float v_obs  = _obs.getVelocity();

    // Filtered speed for the blend decision (tau ~25 ms) to avoid chatter.
    _vel_f += 0.002f * (fabsf(v_hall) - _vel_f);

    // Latch the observer's rotation sign vs the hall once both are clearly
    // moving. Until latched we never hand off (blend stays 0).
    if (!_dir_latched && _vel_f > vel_lo * 0.5f && fabsf(v_obs) > 0.5f) {
      _obs_dir     = (v_hall * v_obs >= 0.0f) ? 1.0f : -1.0f;
      _dir_latched = true;
    }
    float a_obs_d = _obs_dir * a_obs;
    float v_obs_d = _obs_dir * v_obs;

    // Constant offset so (obs + offset) tracks the hall. Slow slew (tau ~50 ms)
    // locks the DC offset without chasing per-rev noise; frozen once observer-led.
    float raw_off = a_hall - a_obs_d;
    if (!_off_init) { offset = raw_off; _off_init = true; }

    // Health: observer velocity must agree with the hall (lag-free, meaningful
    // at any speed). Tolerance widens with speed.
    obs_dv = v_obs_d - v_hall;
    float tol = 0.5f + 0.15f * fabsf(v_hall);
    healthy = _dir_latched && (fabsf(obs_dv) < tol);

    blend = 0.0f;
    if (enabled && healthy) {
      if      (_vel_f <= vel_lo) blend = 0.0f;
      else if (_vel_f >= vel_hi) blend = 1.0f;
      else                       blend = (_vel_f - vel_lo) / (vel_hi - vel_lo);
    }

    offset += (1.0f - blend) * 0.001f * (raw_off - offset);

    float a_hyb = (1.0f - blend) * a_hall + blend * (a_obs_d + offset);
    _v_hyb      = (1.0f - blend) * v_hall + blend * v_obs_d;

    full_rotations = (int32_t)floorf(a_hyb / _2PI);
    angle_prev     = a_hyb - (float)full_rotations * _2PI;
    angle_prev_ts  = _micros();
  }

  // Motor reads getMechanicalAngle() (=angle_prev) for commutation and
  // getVelocity() for the loop; both come from update() above.
  float getVelocity() override { return _v_hyb; }
  float getSensorAngle() override { return angle_prev; }  // not used in main path
  int   needsSearch() override { return 0; }

  float offset = 0.0f;        // hall_angle - dir*obs_angle (continuous)

 private:
  Sensor& _hall;
  float _v_hyb      = 0.0f;
  float _vel_f      = 0.0f;
  float _obs_dir    = 1.0f;
  bool  _dir_latched = false;
  bool  _off_init    = false;
};
