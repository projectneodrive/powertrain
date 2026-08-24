// ============================================================================
//  motor_config.h  —  Everything that depends on the motor fitted and on the
//  power stage tuning: sensor selection, limits, DC-bus thresholds, current
//  sense scaling, saved calibration values and the controller gains.
//
//  Retune per motor. The pin map is in hw_pinout.h, the task rates in
//  plc_config.h.
// ============================================================================
#pragma once
#include <Arduino.h>

// ============================================================================
//  Motor — 26pp hub motor + hall sensors
// ============================================================================
#define CFG_POLE_PAIRS   26
#define CFG_ENC_PPR      600      // (only used if a quadrature enc is fitted)
#define CFG_KV           8.2f     // hub motor KV

// Torque constant (Nm/A). Kt = 8.27 / KV (same relation ODrive uses).
#define CFG_KT           (8.27f / CFG_KV)

// ---------------------------------------------------------------------------
//  Sensor selection (compile-time). Quadrature uses the STM32 hardware timer
//  (TIM3, no interrupts); Hall uses SimpleFOC's interrupt-driven HallSensor on
//  the SAME pins (PB4/PB5/PC9). Hall edge rate is ~2 orders lower than a fast
//  quadrature encoder, so its interrupts don't threaten the scheduler.
//  Override in platformio.ini with e.g. -D SENSOR_TYPE=SENSOR_TYPE_HALL.
// ---------------------------------------------------------------------------
#define SENSOR_TYPE_QUADRATURE  1
#define SENSOR_TYPE_HALL        2

#ifndef SENSOR_TYPE
#define SENSOR_TYPE SENSOR_TYPE_HALL   // hub motor -> hall sensors
#endif

// ---------------------------------------------------------------------------
//  Power / limits — conservative values for bring-up. Tighten per motor.
// ---------------------------------------------------------------------------
#define CFG_VBUS_NOMINAL   24.0f    // driver.voltage_power_supply
#define CFG_PWM_FREQ_HZ    20000    // 20 kHz (matches FOC tick; keeps sense window sane)
#define CFG_VOLT_LIMIT     23.5f    // motor/driver voltage limit (safety)
#define CFG_VOLT_ALIGN     2.0f     // voltage used during initFOC alignment
#define CFG_CURRENT_LIMIT  3.0f     // A (used once current sensing is enabled)
#define CFG_VEL_LIMIT      17.78f   // rad/s

// Hard safety ceilings for the runtime serial settings ('LC'/'LV' from the
// config GUI). A remote client must NEVER be able to ask for an arbitrary
// current/velocity: the accepted value is clamped here.
#define CFG_CURRENT_LIMIT_MAX  20.0f   // A — hard ceiling for 'LC'
#define CFG_VEL_LIMIT_MAX      40.0f   // rad/s — hard ceiling for 'LV'

// ---------------------------------------------------------------------------
//  Regenerated energy management (2 ohm brake resistor on AUX) + DC bus
//  thresholds, for a 24 V nominal bus. Three stages, softest to hardest —
//  this ordering is REQUIRED:
//      BRAKE_VBUS_OFF < BRAKE_VBUS_ON < REGEN_START < REGEN_FULL < OV_TRIP
//   1. chopper on the resistor (BRAKE_VBUS_ON, proportional gain)
//   2. derating of the motor braking torque (REGEN_START -> REGEN_FULL)
//   3. latched over-voltage fault (OV_TRIP)
//  The ordering is what guarantees the resistor ALWAYS gets its chance before
//  we sacrifice braking torque, and before the fault.
// ---------------------------------------------------------------------------
#define CFG_BRAKE_R            2.0f    // ohms, resistor across the AUX terminals
#define CFG_BRAKE_PWM_HZ       20000   // brake PWM (TIM2) — inaudible
// A ceiling < 1.0 is MANDATORY: the high-side FET driver (LM5109B) is
// bootstrapped, and its C70 capacitor only recharges while the low-side FET
// conducts. 0.7 leaves 15 us of recharge per period at 20 kHz — very generous.
// 0.7 * 24²/2 ≈ 200 W peak into the resistor.
#define CFG_BRAKE_MAX_DUTY     0.7f
// Dead time between one FET turning off and the other turning on. Below the AUX
// FETs' real switching time both conduct briefly = a hard short across the bus.
// 500 ns is very conservative (ODrive uses ~240 ns). In center-aligned mode it
// is honoured on BOTH edges (see io_brake.cpp).
#define CFG_BRAKE_DEADTIME_NS  500

// Call rate of the bus-safety block (see PRG_SAFETY): the Vbus measurement is a
// blocking ADC conversion -- no point doing it at 1 kHz (nothing on a battery
// bus moves that fast), and it avoids stealing CPU from PRG_FOC (PRG_SAFETY's
// task is the more urgent one). The safety task runs at 1 kHz, so
// CFG_BUS_SAFETY_HZ must stay an integer divisor of 1000.
#define CFG_BUS_SAFETY_HZ      200
#define CFG_BUS_SAFETY_DIV     (1000 / CFG_BUS_SAFETY_HZ)

// ---------------------------------------------------------------------------
//  Bus power source. Changes the THRESHOLDS ONLY: the source determines whether
//  the braking energy has anywhere to go.
//
//   PSU      : a lab supply CANNOT sink current. All the braking energy goes
//              into the bus capacitance -> it must be dissipated IMMEDIATELY.
//              The brake starts just above the working voltage, and the torque
//              derate stays far above it: the resistor does the work, and we
//              only sacrifice torque as a last resort.
//   BATTERY  : the battery absorbs the regen current (it recharges) as long as
//              it is not full. So we let the bus rise to the end-of-charge
//              voltage BEFORE turning the brake on: energy is recovered first,
//              dissipated second.
//
//  /!\ In BATTERY mode, set BRAKE_VBUS_ON to the pack's REAL end-of-charge
//      voltage (e.g. 6S Li-ion = 4.20 V/cell = 25.2 V). Too high = cell
//      overcharge.
// ---------------------------------------------------------------------------
#define CFG_BUS_SOURCE_PSU      1
#define CFG_BUS_SOURCE_BATTERY  2

#ifndef CFG_BUS_SOURCE
#define CFG_BUS_SOURCE  CFG_BUS_SOURCE_PSU
#endif

#if CFG_BUS_SOURCE == CFG_BUS_SOURCE_PSU
  // Bus measured in operation: 23.5 V (accelerating) to 24.2 V (peak).
  // BRAKE_VBUS_ON must stay above 24.2 so it does not modulate in normal use,
  // and as low as possible so it dissipates before things climb.
  #define CFG_BRAKE_VBUS_ON      24.6f   // V — the chopper engages
  #define CFG_BRAKE_VBUS_OFF     24.2f   // V — it disengages (hysteresis)
  // Braking-torque derate: a safety net only. If the resistor is enough, we
  // never get here.
  #define CFG_VBUS_REGEN_START   26.5f   // V — derate begins
  #define CFG_VBUS_REGEN_FULL    27.5f   // V — braking torque fully cut
#elif CFG_BUS_SOURCE == CFG_BUS_SOURCE_BATTERY
  // 6S Li-ion: 25.2 V for a full pack. Below that, regen recharges the pack and
  // the brake must stay off — otherwise we burn energy we could recover.
  #define CFG_BRAKE_VBUS_ON      25.8f   // V — pack full, nothing left to absorb
  #define CFG_BRAKE_VBUS_OFF     25.4f   // V
  #define CFG_VBUS_REGEN_START   27.0f   // V
  #define CFG_VBUS_REGEN_FULL    28.0f   // V
#else
  #error "CFG_BUS_SOURCE must be CFG_BUS_SOURCE_PSU or CFG_BUS_SOURCE_BATTERY"
#endif

// Chopper gain (duty per volt above BRAKE_VBUS_OFF). 0.1/V -> at 1 V of
// overshoot, duty 0.10 = 0.10 * 24²/2 ≈ 29 W dissipated. Measured on the bench:
// the bus capacitance (~1400 uF) climbs from 24 to 28.5 V on only ~0.4 W of
// regen, so a few tens of watts is plenty to hold the bus. Raise the gain if
// the bus still exceeds BRAKE_VBUS_ON + 2 V.
#define CFG_BRAKE_GAIN         0.1f

#define CFG_VBUS_OV_TRIP       29.0f   // V — latched fault (~10 ms consecutive)

// Max accepted velocity setpoint (rad/s): ~90 % of the no-load speed reachable
// under CFG_VOLT_LIMIT (KV in rpm/V -> *0.10472 for (rad/s)/V). Beyond that the
// setpoint is physically unreachable: the PID saturates and the integrator winds
// up to its maximum without ever converging.
#define CFG_VEL_CMD_MAX    (0.9f * CFG_VOLT_LIMIT * CFG_KV * 0.10472f)

// ---------------------------------------------------------------------------
//  Current-sense hardware — VERIFY THESE ON YOUR CLONE (silkscreen/schematic).
//  Shunt value and DRV8301 amp gain directly scale measured phase current.
//  The DRV_GAIN here MUST equal the gain programmed into DRV8301 CTRL2.
// ---------------------------------------------------------------------------
#define CFG_SHUNT_OHMS     0.005f  //5 mOhm (ODrive 56V); clones vary (verify!)
#define CFG_DRV_GAIN       40.0f    // V/V  (DRV8301: 10/20/40/80 selectable)

// FOC current-loop PID + measurement filter (foc_current torque mode).
// Starting points — expect to bench-tune per motor.
#define CFG_CUR_P          1.0f     // current PID P (V/A)
#define CFG_CUR_I          50.0f   // current PID I
#define CFG_CUR_D          0.0f    // current PID D (rarely used; tunable via JD)
#define CFG_LPF_CUR_TF     0.01f   // current measurement low-pass (s)
#define CFG_CHAR_VOLTAGE   1.0f     // voltage used by characteriseMotor() for R/L

// ---------------------------------------------------------------------------
//  Pre-calibration ("saved" motor params). Run the commissioning procedure once
//  (see docs/Calibration.md), copy the printed numbers here, then set
//  CFG_PRECALIBRATED 1 so the board arms WITHOUT any calibration motion — the
//  compile-time equivalent of ODrive's pre_calibrated. (Flash-runtime saving is
//  a later phase.) Leave 0 to auto-align on each first arm.
// ---------------------------------------------------------------------------
#define CFG_PRECALIBRATED    1            // 1 = use the values below, skip alignment
#define CFG_ZERO_ELEC_ANGLE  5.2154f      // motor.zero_electric_angle (rad), from initFOC
#define CFG_SENSOR_DIRECTION -1           // +1 = CW, -1 = CCW, from initFOC
#define CFG_PHASE_R          4.2093f      // phase resistance (ohm); 0 = leave unset
#define CFG_PHASE_L          4890.65e-6f  // phase inductance (H);   0 = leave unset

// ---------------------------------------------------------------------------
//  Sensorless mode (MESC/Lemming flux observer) above a speed threshold —
//  removes the hall quantization floor at speed. The hall stays active below;
//  the handoff is blended over [VEL_LO, VEL_HI]. Prerequisites: current sense
//  active + CFG_PHASE_R/L filled in (see src/io/HybridSensor.h).
//  COMMISSIONING: keep ENABLE=0, spin the motor, watch 'obsdV' in the telemetry
//  (it must stay ~0 over the whole range) BEFORE setting ENABLE to 1.
// ---------------------------------------------------------------------------
#define CFG_SENSORLESS_ENABLE  1          // 1 = hall->observer handoff above the threshold
#define CFG_SENSORLESS_VEL_LO  5.0f       // rad/s: below this = pure hall
#define CFG_SENSORLESS_VEL_HI  7.0f       // rad/s: above this = pure observer

// ============================================================================
//  CAN (ODrive CANSimple)
// ============================================================================
// These two are read directly by the ESP32 control station as well
// (can_utilities/include/bridge_config.h includes this file), so the bridge
// cannot be pointed at the wrong node or brought up at the wrong bit rate:
// change them here and both ends follow. can_utilities turns CFG_CAN_BAUD into
// a TWAI timing constant and #errors on a rate it has no constant for.
#define CFG_CAN_NODE_ID   0
#define CFG_CAN_BAUD      500000     // bit rate, both ends of the bus (500 kbit/s)
#define CFG_WATCHDOG_MS   0          // CAN setpoint timeout; 0 = disabled.
                                     // Set e.g. 250 for an e-bike so that losing
                                     // the CAN master disarms the motor.
#define CFG_VBUS_DIV      19.0f      // Vbus ADC divider ratio — verify against your board

// ============================================================================
//  Motion controller defaults (velocity / position modes over CAN)
// ============================================================================
// In foc_current the velocity PID's output is a current (A). Retune per motor.
// Too high a P amplifies the hall measurement noise -> Iq oscillates and the
// resulting regen jolts can fault the bus (seen from P=1.0 upwards); raise in
// small steps (+0.1).
#define CFG_VEL_P        0.1f        // A/(rad/s)
// I sets the cruise current (Ti=P/I). Too low -> stick-slip on breakaway (the
// rotor sticks then releases). Raise in steps if low-speed starts catch, lower
// it if it overshoots or oscillates in steady state.
#define CFG_VEL_I        0.05f       // A/(rad·s⁻¹·s)
// D differentiates the hall noise straight into Iq -> keep it very low (the
// multi-edge smoothing in HallSensorSmoothVel already treats that noise at the
// source).
//
// /!\ SimpleFOC computes the D term as D*(error - error_prev)/dt with dt = 1 ms
// (the motion rate), so the EFFECTIVE gain on a sample-to-sample velocity change
// is D/dt: 0.0001 here means 0.1 A per rad/s of change. At low speed the hall
// velocity is quantisation-dominated -- at 1 rad/s only ~1.2 hall edges fall
// inside CFG_HALL_VEL_WINDOW -- so the estimate updates in STEPS, and every step
// is differentiated into a current spike. Ten times this value was carried in by
// a transcription slip when board_config.h was split into config/ (commit
// 84c3946), and it made the motor rough at low speed for every build after it:
// Iq tracked the velocity CHANGE rather than the velocity ERROR. If you raise
// it, check that correlation before believing the result.
#define CFG_VEL_D        0.0001f
// Max slope of the PID's output CURRENT (A/s). A wide ramp also speeds up the
// torque reversal when braking -> a faster bus voltage spike.
#define CFG_VEL_RAMP     30.0f      // PID output ramp (A/s)
// Acceleration limit on the velocity SETPOINT (rad/s²) — distinct from
// CFG_VEL_RAMP (current): it smooths the target. Without a ramp, a step (e.g.
// V5->V10) makes the bench supply dip then overshoot (regen -> OV_TRIP). Too
// fast, and the ramp stopping dead at the target excites a speed overshoot on
// arrival (~5 Hz ringing) which regenerates and makes the brake flicker = a
// stutter. 10 = 0->10 rad/s in ~1 s. Lower it (6-8) if arrival still stutters,
// raise it for a livelier response. 0 = direct step.
#define CFG_VEL_ACCEL    20.0f      // rad/s²  (0 = no setpoint ramp)
#define CFG_POS_P        1.0f       // position P gain ((rad/s)/rad)
#define CFG_POS_I        0.0f       // position I gain (tunable via PI; usually 0)
#define CFG_POS_D        0.0f       // position D gain (tunable via PD; usually 0)
// Low-pass on the measured velocity (s). Too large adds loop delay -> sustained
// oscillation (a ~5 Hz limit cycle was seen at 0.15 s). The main smoothing is
// done by CFG_HALL_VEL_WINDOW; this filter stays light.
#define CFG_LPF_VEL_TF   0.02f       // velocity low-pass (s)

// Hall velocity averaging window (s) -- see src/io/HallSensorSmoothVel.h. Forces
// Sensor::getVelocity() to span several hall edges per computation (cancelling
// the sector-to-sector spacing error). Inter-edge period ~2π/(pp*6*vel): ~20 ms
// at 2 rad/s, ~8 ms at 5 rad/s; 0.05 s covers ~2.5 edges at 2 rad/s. Too large =
// loop delay, too small = quantization (low-speed stutter).
#define CFG_HALL_VEL_WINDOW  0.05f

// ---------------------------------------------------------------------------
//  Hall transition angle calibration (anti commutation ripple).
//  SimpleFOC assumes hall sectors of exactly 60° elec; the real placement is
//  irregular -> a wrong commutation angle in each sector -> torque ripple at
//  every speed. See src/io/HallSensorSmoothVel.h.
//  Procedure: motor DISARMED -> serial command 'H' (open-loop spin, ~10 s) ->
//  copy the 6 printed offsets into CFG_HALL_CAL_OFFSETS -> set
//  CFG_HALL_PRECALIBRATED to 1 -> rebuild. Otherwise the offsets only live in RAM.
// ---------------------------------------------------------------------------
#define CFG_HALL_CAL_VOLTAGE     2.0f    // V, voltage of the open-loop spin
#define CFG_HALL_CAL_ELEC_SPEED  8.0f    // rad/s elec (~0.31 rad/s mech at 26pp)
#define CFG_HALL_CAL_REVS        12      // elec revs swept (first 2 discarded)
#define CFG_HALL_PRECALIBRATED   1       // 1 = load CFG_HALL_CAL_OFFSETS at boot
// Mechanical offsets (rad) per hall sector 0..5, produced by 'H' (direction
// included). Sectors 0..5 in deg elec: +0.92 -1.18 +0.46 +0.36 -1.33 +0.77.
#define CFG_HALL_CAL_OFFSETS  { -0.0006175f, 0.0007926f, -0.0003101f, -0.0002395f, 0.0008929f, -0.0005184f }
