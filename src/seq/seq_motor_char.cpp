// ============================================================================
//  seq_motor_char.cpp — see seq_motor_char.h.
// ============================================================================
#include "seq/seq_motor_char.h"
#include "io/io_motor.h"
#include "gvl/gvl.h"
#include "config/motor_config.h"

namespace seq {

void motorCharacterise(bool safe) {
  // characteriseMotor() takes over setPhaseVoltage, so it requires the motor
  // DISARMED. Report every refusal: an 'M' sent while armed used to be ignored
  // silently, with the request flag left set.
  if (gvl::M.foc_ready || gvl::AXIS.armed) {
    Serial.println("[!] M: désarmer d'abord (envoyer 'I'), puis 'M'.");
    return;
  }
  if (!gvl::IN.isense_ok) {
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

} // namespace seq
