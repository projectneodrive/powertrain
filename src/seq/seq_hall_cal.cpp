// ============================================================================
//  seq_hall_cal.cpp — see seq_hall_cal.h.
// ============================================================================
#include "seq/seq_hall_cal.h"

#if SENSOR_TYPE == SENSOR_TYPE_HALL

#include "io/io_motor.h"
#include "gvl/gvl.h"

namespace seq {

bool hallCalibrate() {
  using io::motor::motor;
  using io::motor::sensor;

  const float Uq     = CFG_HALL_CAL_VOLTAGE;
  const float wel    = CFG_HALL_CAL_ELEC_SPEED;   // rad/s elec
  const int   revs   = CFG_HALL_CAL_REVS;
  const float dt     = 0.0005f;                    // 500 us / step
  const float dtheta = wel * dt;
  const float pp     = (float)CFG_POLE_PAIRS;

  Serial.println("Hall cal: spin boucle ouverte ~10s (moteur doit tourner lentement)...");
  io::motor::enableStage();

  // Lock the rotor at theta=0 and let the transient settle, otherwise the first
  // transitions captured would be corrupted by the oscillation.
  for (int i = 0; i < 500; i++) { motor.setPhaseVoltage(Uq, 0.0f, 0.0f); delay(1); }

  float sacc[6] = {0}, cacc[6] = {0};   // circular mean of the residual per sector
  int   cnt[6]  = {0};
  int   seq_up = 0, seq_down = 0;       // vote on the direction (sector vs theta)
  int   dir = 0;
  float theta = 0.0f;
  const float theta_end = (float)revs * _2PI;
  int8_t prev = sensor.electric_sector;

  while (theta < theta_end) {
    theta += dtheta;
    motor.setPhaseVoltage(Uq, 0.0f, theta);
    delayMicroseconds(500);

    int8_t s = sensor.electric_sector;                 // live (ISR)
    if (s != prev && s >= 0 && s <= 5 && prev >= 0 && prev <= 5) {
      int d = s - prev;                                // sector progression direction
      if (d == 1 || d == -5)      seq_up++;
      else if (d == -1 || d == 5) seq_down++;
      prev = s;
    }
    if (theta < 2.0f * _2PI) continue;                 // 2 revs to reach steady state
    if (dir == 0) dir = (seq_up >= seq_down) ? 1 : -1; // direction frozen once settled

    // Electrical residual = theta - dir*pp*angle_prev, reduced mod 2PI. The
    // full_rotations term vanishes mod 2PI (pp*2PI == 0), so reading angle_prev
    // (getMechanicalAngle, an atomic float) is enough and avoids any tearing.
    int8_t sc = sensor.electric_sector;
    if (sc < 0 || sc > 5) continue;
    float r = theta - (float)dir * pp * sensor.getMechanicalAngle();
    sacc[sc] += sinf(r);
    cacc[sc] += cosf(r);
    cnt[sc]++;
  }

  motor.setPhaseVoltage(0.0f, 0.0f, theta);
  motor.disable();

  for (int s = 0; s < 6; s++) {
    if (cnt[s] < 5) {
      Serial.println("[-] Hall cal ÉCHEC : secteur peu/non vu (le moteur a-t-il tourné ? monter CFG_HALL_CAL_VOLTAGE).");
      return false;
    }
  }

  // Mean (circular) residual per sector, then remove the global mean.
  float res[6], ms = 0.0f, mc = 0.0f;
  for (int s = 0; s < 6; s++) {
    res[s] = atan2f(sacc[s], cacc[s]);   // [-pi, pi]
    ms += sinf(res[s]); mc += cosf(res[s]);
  }
  float mean = atan2f(ms, mc);

  Serial.print("Hall cal OK (dir="); Serial.print(dir);
  Serial.println("). CFG_HALL_CAL_OFFSETS =");
  Serial.println("{");
  for (int s = 0; s < 6; s++) {
    float off_e = res[s] - mean;                 // mean-zero electrical residual
    while (off_e >  _PI) off_e -= _2PI;
    while (off_e < -_PI) off_e += _2PI;
    // Mechanical delta to add to angle_prev: dir*off_e/pp (derivation in
    // motor_config.h).
    float off_m = (float)dir * off_e / pp;
    sensor.sector_offset[s] = off_m;
    Serial.print("  "); Serial.print(off_m, 7); Serial.print("f,");
    Serial.print("  // secteur "); Serial.print(s);
    Serial.print("  ("); Serial.print(off_e * 180.0f / _PI, 2); Serial.println(" deg elec)");
  }
  Serial.println("};");
  Serial.println("-> copier dans config/motor_config.h puis CFG_HALL_PRECALIBRATED=1 pour figer.");

  sensor.offsets_active = true;
  // Force a re-initFOC on the next arm so zero_electric_angle is recomputed
  // with the offsets active.
  gvl::M.calibrated = false;
  return true;
}

} // namespace seq

#endif // SENSOR_TYPE_HALL
