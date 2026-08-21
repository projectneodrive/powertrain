// ============================================================================
//  io_motor.cpp — see io_motor.h.
// ============================================================================
#include "io/io_motor.h"
#include "io/io_gate.h"
#include "state.h"
#include "config/tasks_config.h"
#include "current_sense/hardware_specific/stm32/stm32_mcu.h"  // Stm32CurrentSenseParams

namespace io {
namespace motor {

BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_M0_INH_A, PIN_M0_INL_A,
                                       PIN_M0_INH_B, PIN_M0_INL_B,
                                       PIN_M0_INH_C, PIN_M0_INL_C, PIN_EN_GATE);
BLDCMotor motor = BLDCMotor(CFG_POLE_PAIRS);

#if SENSOR_TYPE == SENSOR_TYPE_HALL
HallSensorSmoothVel sensor = HallSensorSmoothVel(PIN_ENC_A, PIN_ENC_B, PIN_ENC_Z, CFG_POLE_PAIRS);
static void doHallA() { sensor.handleA(); }
static void doHallB() { sensor.handleB(); }
static void doHallC() { sensor.handleC(); }
// Interpolates the angle between two hall edges (60 deg elec of resolution
// otherwise).
SmoothingSensor smooth = SmoothingSensor(sensor, motor);
// Hybrid hall + sensorless observer (hands over at speed). Transparent (= pure
// hall) as long as hybrid.enabled is false. See HybridSensor.h.
HybridSensor hybrid = HybridSensor(smooth, motor);
Sensor& foc_sensor = hybrid;
#else
STM32HWEncoder sensor = STM32HWEncoder(CFG_ENC_PPR, PIN_ENC_A, PIN_ENC_B);
Sensor& foc_sensor = sensor;
#endif

LowsideCurrentSense current_sense =
    LowsideCurrentSense(CFG_SHUNT_OHMS, CFG_DRV_GAIN, _NC, PIN_M0_IB, PIN_M0_IC);

// ---------------------------------------------------------------------------
ADC_TypeDef* currentSenseAdc() {
  if (!state::at_boot.isense_ok || !current_sense.params) return nullptr;
  ADC_HandleTypeDef *h = ((Stm32CurrentSenseParams *)current_sense.params)->adc_handle;
  return h ? h->Instance : nullptr;
}

// ---------------------------------------------------------------------------
void enableStage() {
  motor.enable();
  delay(5); // Setup time for DRV8301 SPI stability
  gate::setGain();
}

// ---------------------------------------------------------------------------
void init() {
  // ---- Sensor ----
  sensor.init();
#if SENSOR_TYPE == SENSOR_TYPE_HALL
  sensor.enableInterrupts(doHallA, doHallB, doHallC);
  // Force multi-edge velocity averaging (see HallSensorSmoothVel.h) instead
  // of Sensor's library default 100us (effectively single-edge at our poll rate).
  sensor.min_elapsed_time = CFG_HALL_VEL_WINDOW;
  // Pre-calibrated hall sector angle corrections (see motor_config.h + the 'H'
  // serial command). Otherwise the offsets stay zero/inactive until an 'H' is
  // run in-session.
  if (CFG_HALL_PRECALIBRATED) {
    const float hall_cal[6] = CFG_HALL_CAL_OFFSETS;
    for (int i = 0; i < 6; i++) sensor.sector_offset[i] = hall_cal[i];
    sensor.offsets_active = true;
    Serial.println("Hall sector offsets: PRECALIBRATED (CFG_HALL_CAL_OFFSETS actifs)");
  }
#endif
  motor.linkSensor(&foc_sensor);

  // ---- Driver ----
  driver.voltage_power_supply = CFG_VBUS_NOMINAL;
  driver.pwm_frequency        = CFG_PWM_FREQ_HZ;
  driver.voltage_limit        = CFG_VOLT_LIMIT;
  if (!driver.init()) { Serial.println("[-] driver.init FAILED"); while (1); }
  motor.linkDriver(&driver);

  // ---- Low-side current sense ----
  current_sense.linkDriver(&driver);
  // skip_align=true SKIPS initFOC's shunt polarity/pin verification. We only
  // skip it once the configuration has been validated and frozen
  // (CFG_PRECALIBRATED): an inverted polarity in foc_current is positive
  // feedback (runaway).
  current_sense.skip_align = (CFG_PRECALIBRATED != 0);
  state::at_boot.isense_ok = (current_sense.init() == 1);
  if (state::at_boot.isense_ok) {
    motor.linkCurrentSense(&current_sense);
    // Closed current loop: the velocity PID now outputs amps and
    // motor.current_limit becomes effective (in voltage mode it was ignored).
    motor.torque_controller = TorqueControlType::foc_current;
    motor.PID_current_q.P = CFG_CUR_P; motor.PID_current_d.P = CFG_CUR_P;
    motor.PID_current_q.I = CFG_CUR_I; motor.PID_current_d.I = CFG_CUR_I;
    motor.LPF_current_q.Tf = CFG_LPF_CUR_TF;
    motor.LPF_current_d.Tf = CFG_LPF_CUR_TF;
    Serial.println("Current sense OK -> foc_current torque control");
  } else {
    motor.torque_controller = TorqueControlType::voltage;
    Serial.println("[!] current_sense.init FAILED -> voltage-torque fallback");
  }

  // ---- Motor parameters ----
  motor.voltage_limit        = CFG_VOLT_LIMIT;
  motor.current_limit        = CFG_CURRENT_LIMIT;
  motor.velocity_limit       = CFG_VEL_LIMIT;
  motor.controller           = MotionControlType::velocity;
  motor.foc_modulation       = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = CFG_VOLT_ALIGN;
  motor.motion_downsample    = MOTION_DOWNSAMPLE;
  motor.PID_velocity.P       = CFG_VEL_P;
  motor.PID_velocity.I       = CFG_VEL_I;
  motor.PID_velocity.D       = CFG_VEL_D;
  motor.PID_velocity.output_ramp = CFG_VEL_RAMP;
  motor.P_angle.P            = CFG_POS_P;
  motor.LPF_velocity.Tf      = CFG_LPF_VEL_TF;

  if (CFG_PHASE_R > 0.0f) motor.phase_resistance = CFG_PHASE_R;
  if (CFG_PHASE_L > 0.0f) motor.phase_inductance = CFG_PHASE_L;
  motor.KV_rating = CFG_KV;

#if SENSOR_TYPE == SENSOR_TYPE_HALL
  // Sensorless observer: flux_linkage (Wb) derived from KV/pp. The observer's
  // constructor runs before setup() (KV_rating not set yet) -> do it here.
  hybrid._obs.flux_linkage = 60.0f / (_SQRT3 * _PI * CFG_KV * (float)CFG_POLE_PAIRS * 2.0f);
  hybrid._obs.min_elapsed_time = CFG_HALL_VEL_WINDOW;  // same window as the hall
                                          // -> obsdV is a true tracking error,
                                          //    not a window artefact, and v_obs
                                          //    is smoothed
  hybrid.vel_lo  = CFG_SENSORLESS_VEL_LO;
  hybrid.vel_hi  = CFG_SENSORLESS_VEL_HI;
  hybrid.enabled = (CFG_SENSORLESS_ENABLE != 0) && state::at_boot.isense_ok;  // needs the current sense
  hybrid.initObserver();
#endif

  if (!motor.init()) { Serial.println("[-] motor.init FAILED"); while (1); }
  motor.disable(); // Force initial safe state
}

} // namespace motor
} // namespace io
