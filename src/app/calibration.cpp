// ============================================================================
//  seq_hall_cal.cpp — see seq_hall_cal.h.
// ============================================================================
#include "app.h"

#if SENSOR_TYPE == SENSOR_TYPE_HALL

#include "io/io_motor.h"
#include "state.h"

namespace app {
namespace calibration {

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

  // Evidence, so a failure says WHICH thing went wrong instead of guessing at
  // the two most likely. "Sector rarely seen" has two very different causes -
  // a rotor that never turned, and halls that are not being read - and they
  // need opposite fixes.
  int   transitions = 0;                        // sector changes observed
  int   bad_sector  = 0;                        // reads outside 0..5
  // getAngle() is the multi-turn mechanical angle; full_rotations itself is
  // protected in Sensor, and this is what it is for.
  const float mech_start = sensor.getAngle();
  uint32_t next_report = 2000;                  // ms

  while (theta < theta_end) {
    theta += dtheta;
    motor.setPhaseVoltage(Uq, 0.0f, theta);
    delayMicroseconds(500);

    // `prev` RESYNCS on every valid read, and is never left holding an invalid
    // one. It used to be updated only inside the transition test, which also
    // required prev to be valid - so a single invalid sector (hall state 000 or
    // 111) latched prev at -1 permanently, and from then on no transition was
    // ever counted, seq_up/seq_down stayed 0, and dir silently defaulted to +1
    // whatever the rotor did. A board that starts the spin on an invalid state
    // never had working direction detection at all.
    int8_t s = sensor.electric_sector;                 // live (ISR)
    if (s < 0 || s > 5) {
      bad_sector++;
    } else {
      if (prev >= 0 && prev <= 5 && s != prev) {
        int d = s - prev;                              // sector progression direction
        if (d == 1 || d == -5)      seq_up++;
        else if (d == -1 || d == 5) seq_down++;
        transitions++;
      }
      prev = s;
    }

    // A short progress line every 2 s. Kept short and rare on purpose: this
    // task is below the FOC task so a print cannot starve commutation, but it
    // does pause the theta sweep, and a long pause lets the rotor slip.
    if (millis() >= next_report) {
      next_report += 2000;
      Serial.print("  ... elec rev "); Serial.print(theta / _2PI, 1);
      Serial.print("/");               Serial.print(revs);
      Serial.print("  sector=");       Serial.print(s);
      Serial.print("  transitions=");  Serial.print(transitions);
      Serial.print("  mech=");
      Serial.print(sensor.getAngle() - mech_start, 3);
      Serial.println(" rad");
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

  const float mech_travel = sensor.getAngle() - mech_start;
  bool thin = false;
  for (int s = 0; s < 6; s++) if (cnt[s] < 5) thin = true;

  if (thin) {
    Serial.println("[-] Hall cal ECHEC : un secteur au moins n'a pas ete vu.");
    Serial.print  ("    samples/secteur :");
    for (int s = 0; s < 6; s++) { Serial.print(" "); Serial.print(cnt[s]); }
    Serial.println();
    Serial.print  ("    transitions="); Serial.print(transitions);
    Serial.print  ("  up=");            Serial.print(seq_up);
    Serial.print  ("  down=");          Serial.print(seq_down);
    Serial.print  ("  secteurs_invalides="); Serial.println(bad_sector);
    Serial.print  ("    rotation mecanique mesuree = "); Serial.print(mech_travel, 3);
    Serial.print  (" rad  (attendu ~");
    Serial.print((float)revs * _2PI / pp, 3); Serial.println(" rad)");
    Serial.println("    LECTURE :");
    Serial.println("      transitions ~0 et mecanique ~0  -> le rotor n'a pas tourne :");
    Serial.println("        monter CFG_HALL_CAL_VOLTAGE (couple), ou baisser");
    Serial.println("        CFG_HALL_CAL_ELEC_SPEED (le rotor ne suit pas la rampe).");
    Serial.println("      mecanique correcte mais un secteur a 0 -> un fil hall est mort :");
    Serial.println("        verifier PB4/PB5/PC9 - la commutation marche encore sur 5 secteurs.");
    Serial.println("      secteurs_invalides > 0 -> etat hall 000 ou 111 : cablage/alim hall.");
    return false;
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
  state::control.calibrated = false;
  return true;
}

// ---------------------------------------------------------------------------
void characteriseMotor(bool safe) {
  // characteriseMotor() takes over setPhaseVoltage, so it requires the motor
  // DISARMED. Report every refusal: an 'M' sent while armed used to be ignored
  // silently, with the request flag left set.
  if (state::control.foc_ready || state::axis.armed) {
    Serial.println("[!] M: désarmer d'abord (envoyer 'I'), puis 'M'.");
    return;
  }
  if (!state::at_boot.isense_ok) {
    Serial.println("[!] M: current-sense non initialisé (voir boot).");
    return;
  }
  if (!safe) {
    Serial.println("[!] M: faute active -> 'C' pour l'effacer d'abord.");
    return;
  }

  Serial.println("Characterising motor (R/L)... (quelques secondes, moteur immobile)");
  io::motor::enableStage();
  int rc = io::motor::motor.characteriseMotor(CFG_CHAR_VOLTAGE);
  io::motor::motor.disable();

  if (rc == 0) {
    Serial.print("  R = "); Serial.print(io::motor::motor.phase_resistance, 4);
    Serial.print(" ohm   L = "); Serial.print(io::motor::motor.phase_inductance * 1e6f, 2);
    Serial.println(" uH");
  } else {
    // 1=CS not init, 2=voltage<=0, 3=current too low (raise CFG_CHAR_VOLTAGE), 4=R<=0
    Serial.print("[!] Characterise échec, code "); Serial.println(rc);
  }
}

}  // namespace calibration
}  // namespace app

#endif // SENSOR_TYPE_HALL
