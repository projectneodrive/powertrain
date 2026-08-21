// ============================================================================
//  control.cpp — the axis state machine. See app.h.
//
//  Scan order matters and mirrors the original: the reboot/clear requests and
//  the gain updates are serviced first (they must work even while faulted),
//  then the blocking commissioning sequences take the scan for themselves if
//  requested, then arming, then the runtime setpoint.
// ============================================================================
#include "app.h"

#include "config/motor_config.h"
#include "config/tasks_config.h"
#include "io/io_brake.h"
#include "io/io_gate.h"
#include "io/io_motor.h"
#include "state.h"
#include "util/timers.h"

using namespace odcan;

namespace app {
namespace control {
namespace {

// Ramp the velocity setpoint rather than stepping it. Without a ramp, a step
// (V5 -> V10) makes a bench supply dip then overshoot, and the regen spike can
// trip the bus. The ramp's state IS state::control.active_target — process data
// zeroed on disarm and written directly by the position and torque modes — so a
// helper holding its own copy would go stale across those transitions and a
// re-arm would resume slewing from a setpoint nobody asked for.
float velocityRamp(float current, float cmd, uint32_t scan_ms) {
  if (CFG_VEL_ACCEL <= 0.0f) return cmd;
  float step = CFG_VEL_ACCEL * (scan_ms * 0.001f);
  return current + util::limit(-step, cmd - current, step);
}

// ---------------------------------------------------------------------------
}  // namespace

void init() {
  auto& AX = state::axis;
  AX.armed         = false;
  AX.estop         = false;
  AX.control_mode  = CTRL_TORQUE;
  AX.input_torque  = 0.0f;
  AX.input_vel     = 0.0f;
  AX.vel_limit     = CFG_VEL_LIMIT;
  AX.current_limit = CFG_CURRENT_LIMIT;
  AX.pos_gain      = CFG_POS_P;   // mirror of the position P gain (cmd G/PP)
  AX.pos_int_gain  = CFG_POS_I;
  AX.pos_d_gain    = CFG_POS_D;
  AX.cur_p_gain    = CFG_CUR_P;   // mirrors of the current gains (V/A, no Kt)
  AX.cur_int_gain  = CFG_CUR_I;
  AX.cur_d_gain    = CFG_CUR_D;
  // Velocity gain mirrors in Nm/(rad/s): the exact inverse of how they are
  // applied in applyPendingGains() (the CFG_* are in A/(rad/s), or in V in the
  // voltage fallback).
  {
    float k = state::at_boot.isense_ok ? CFG_KT : 1.0f;
    AX.vel_gain     = CFG_VEL_P * k;
    AX.vel_int_gain = CFG_VEL_I * k;
    AX.vel_d_gain   = CFG_VEL_D * k;
  }
  AX.last_setpoint_ms = millis();
}

// ---------------------------------------------------------------------------
namespace {
void applyPendingGains() {
  auto& AX    = state::axis;
  auto& motor = io::motor::motor;

  // --- Velocity PID gains (CAN Set_Vel_Gains, or serial KP/KI/KD) ---
  if (AX.req_vel_gains) {
    AX.req_vel_gains = false;
    // The mirrors are in Nm/(rad/s); in foc_current the PID outputs amps, hence
    // the /Kt. In the voltage fallback the value is applied as-is (volts).
    float k = state::at_boot.isense_ok ? (1.0f / CFG_KT) : 1.0f;
    motor.PID_velocity.P = AX.vel_gain     * k;
    motor.PID_velocity.I = AX.vel_int_gain * k;
    motor.PID_velocity.D = AX.vel_d_gain   * k;
    Serial.print("[PID vel] P="); Serial.print(motor.PID_velocity.P, 4);
    Serial.print(" I=");          Serial.print(motor.PID_velocity.I, 4);
    Serial.print(" D=");          Serial.println(motor.PID_velocity.D, 5);
  }

  // --- Current PID gains (serial JP/JI/JD): applied as-is to both axes (q and
  // d) of the SimpleFOC current regulator. In V/A, no Kt conversion.
  if (AX.req_cur_gains) {
    AX.req_cur_gains = false;
    motor.PID_current_q.P = AX.cur_p_gain;   motor.PID_current_d.P = AX.cur_p_gain;
    motor.PID_current_q.I = AX.cur_int_gain; motor.PID_current_d.I = AX.cur_int_gain;
    motor.PID_current_q.D = AX.cur_d_gain;   motor.PID_current_d.D = AX.cur_d_gain;
    Serial.print("[PID cur] P="); Serial.print(motor.PID_current_q.P, 4);
    Serial.print(" I=");          Serial.print(motor.PID_current_q.I, 4);
    Serial.print(" D=");          Serial.println(motor.PID_current_q.D, 5);
  }

  // --- Position I/D gains (serial PI/PD). The P (P_angle.P) is applied every
  // scan in updateSetpoint(); only I and D are set here (rarely useful).
  if (AX.req_pos_gains) {
    AX.req_pos_gains = false;
    motor.P_angle.I = AX.pos_int_gain;
    motor.P_angle.D = AX.pos_d_gain;
    Serial.print("[PID pos] P="); Serial.print(motor.P_angle.P, 4);
    Serial.print(" I=");          Serial.print(motor.P_angle.I, 4);
    Serial.print(" D=");          Serial.println(motor.P_angle.D, 5);
  }
}

// ---------------------------------------------------------------------------
bool runPendingSequence(bool safe) {
  auto& AX = state::axis;

  // Always consume the flag, and always answer — see seq_motor_char.h.
  if (AX.req_characterise) {
    AX.req_characterise = false;
    calibration::characteriseMotor(safe);
    return true;
  }

#if SENSOR_TYPE == SENSOR_TYPE_HALL
  if (state::req.hall_cal && !state::control.foc_ready) {
    state::req.hall_cal = false;
    if (safe && !AX.armed) {
      calibration::hallCalibrate();
      io::motor::motor.disable();
    } else {
      Serial.println("[!] Hall cal exige moteur désarmé + état sain (envoyer 'I' puis 'H').");
    }
    return true;
  }
#endif

  return false;
}

// ---------------------------------------------------------------------------
void updateSetpoint() {
  auto& AX    = state::axis;
  auto& motor = io::motor::motor;

  switch (AX.control_mode) {
    case CTRL_VELOCITY: {
      motor.controller = MotionControlType::velocity;
      // Bound the setpoint to what is reachable: past that the PID saturates
      // and the integrator winds up without ever converging.
      float vmax = (motor.velocity_limit < CFG_VEL_CMD_MAX) ? motor.velocity_limit
                                                            : CFG_VEL_CMD_MAX;
      float cmd = util::limit(-vmax, AX.input_vel, vmax);
      state::control.active_target = velocityRamp(state::control.active_target, cmd, SCAN_MS_COMMS);
      break;
    }
    case CTRL_POSITION:
      motor.controller     = MotionControlType::angle;
      state::control.active_target = AX.input_pos;
      break;
    case CTRL_TORQUE:
    case CTRL_VOLTAGE:
    default:
      motor.controller = MotionControlType::torque;
      state::control.active_target =
          state::at_boot.isense_ok ? (AX.input_torque / CFG_KT) : AX.input_torque;
      break;
  }

  // SimpleFOC setters: they also propagate the limits into the internal PIDs
  // (in foc_current, PID_velocity.limit = current_limit).
  if (AX.vel_limit     > 0.0f) motor.updateVelocityLimit(AX.vel_limit);
  if (AX.current_limit > 0.0f) motor.updateCurrentLimit(AX.current_limit);
  if (AX.pos_gain      > 0.0f) motor.P_angle.P = AX.pos_gain;

  if (CFG_WATCHDOG_MS > 0 && (millis() - AX.last_setpoint_ms) > CFG_WATCHDOG_MS) {
    AX.raiseError(ERR_WATCHDOG_EXPIRED);
    AX.armed = false;
  }
}

// ---------------------------------------------------------------------------
}  // namespace

void update() {
  auto& AX    = state::axis;
  auto& motor = io::motor::motor;

  if (AX.req_reboot) {
    io::brake::off();
    motor.disable();
    io::gate::disable();
    NVIC_SystemReset();
  }

  // NOTE: req_clear_errors is consumed by app::safety, which owns the fault
  // latch. Clearing it here would let this task erase a fault that safety
  // latched microseconds ago and re-arm into a live over-voltage.

  applyPendingGains();

  // Global safety, simplified (PRG_SAFETY owns the hardware).
  bool safe = !AX.estop && !state::safety.fault;

  if (runPendingSequence(safe)) return;

  bool want = AX.armed && safe;

  // --- DISARM ---
  if (!want && state::control.foc_ready) {
    state::control.foc_ready     = false;
    state::control.active_target = 0.0f;
    motor.disable();
  }

  // --- ARM & CALIBRATE ---
  if (want && !state::control.foc_ready) {
    io::motor::enableStage();
    if (!state::control.calibrated) {
      if (CFG_PRECALIBRATED) {
        motor.sensor_direction    = (CFG_SENSOR_DIRECTION >= 0) ? Direction::CW : Direction::CCW;
        motor.zero_electric_angle = CFG_ZERO_ELEC_ANGLE;
      }
      if (motor.initFOC()) {
        state::control.calibrated = true;
        Serial.print("initFOC OK | CFG_SENSOR_DIRECTION=");
        Serial.print(motor.sensor_direction == Direction::CW ? 1 : -1);
        Serial.print("  CFG_ZERO_ELEC_ANGLE=");
        Serial.println(motor.zero_electric_angle, 4);
      } else {
        Serial.println("[-] initFOC FAILED -> disarm (voir logs MOT: ci-dessus)");
        AX.raiseError(ERR_ENCODER_FAILED);
        AX.armed = false;
        motor.disable();
        return;
      }
    }
    state::control.foc_ready = true;
  }

  if (state::control.foc_ready) updateSetpoint();
  else                  state::control.active_target = 0.0f;
}

}  // namespace control
}  // namespace app
