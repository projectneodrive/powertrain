// ============================================================================
//  board_config.h  —  Single source of truth for the ODrive v3.6 (MKS clone)
//  SimpleFOC firmware. Pins, motor config, limits and RTOS timing live here.
//
//  Target: STM32F405RGT6 @ 168 MHz, DRV8301 gate driver, single channel (M0).
//  Pin map reconciled against the authoritative ODrive v3.6 hardware:
//    - DRV8301 SPI chip-select is PC13 (the PC4/PC5 in test_driver_on.cc is wrong)
//    - M0 encoder/hall is PB4/PB5 (+PC9) = TIM3  (PB6/PB7 in the speed tests = M1)
// ============================================================================
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Gate driver (DRV8301) — TIM1 6-PWM
// ---------------------------------------------------------------------------
#define PIN_M0_INH_A   PA8    // TIM1_CH1
#define PIN_M0_INH_B   PA9    // TIM1_CH2
#define PIN_M0_INH_C   PA10   // TIM1_CH3
#define PIN_M0_INL_A   PB13   // TIM1_CH1N
#define PIN_M0_INL_B   PB14   // TIM1_CH2N
#define PIN_M0_INL_C   PB15   // TIM1_CH3N
#define PIN_EN_GATE    PB12   // DRV8301 enable (toggle LOW->HIGH to reset)
#define PIN_N_FAULT    PD2    // DRV8301 nFAULT (LOW = fault), shared M0/M1

// ---------------------------------------------------------------------------
//  DRV8301 configuration SPI (SPI3) — used from Phase 4 onward
// ---------------------------------------------------------------------------
#define PIN_DRV_SCK    PC10   // SPI3_SCK
#define PIN_DRV_MISO   PC11   // SPI3_MISO
#define PIN_DRV_MOSI   PC12   // SPI3_MOSI
#define PIN_M0_CS      PC13   // DRV8301 M0 chip-select (active low)
#define PIN_M1_CS      PC14   // DRV8301 M1 chip-select (unused, held high)

// ---------------------------------------------------------------------------
//  Phase current sense (ADC2) + Vbus (ADC1) — used from Phase 4 onward
// ---------------------------------------------------------------------------
#define PIN_M0_IB      PC0    // ADC2_IN10  (phase B shunt amp)
#define PIN_M0_IC      PC1    // ADC2_IN11  (phase C shunt amp; A reconstructed)
#define PIN_VBUS       PA6    // ADC1_IN6   (bus voltage divider)

// ---------------------------------------------------------------------------
//  Encoder / Hall (M0). Quadrature -> PB4/PB5 as TIM3 AF.
//  Hall -> PB4/PB5/PC9 as EXTI GPIO. Only one sensor active at a time.
// ---------------------------------------------------------------------------
#define PIN_ENC_A      PB4    // TIM3_CH1 / Hall A
#define PIN_ENC_B      PB5    // TIM3_CH2 / Hall B
#define PIN_ENC_Z      PC9    // encoder index / Hall C

// ---------------------------------------------------------------------------
//  Demi-pont AUX (résistance de freinage), gate driver dédié — TIM2.
//  Topologie ODrive v3.6 : résistance entre DC+ et le point milieu. Seul le
//  FET BAS (AUX_L) dissipe ; le FET HAUT est tenu BAS en permanence, sa diode
//  de corps assure la roue libre vers DC+. Ne JAMAIS piloter les deux.
//  Pins de la v3.6 de référence — à vérifier sur le clone si le frein ne
//  réagit pas (schéma/continuité vers le driver du demi-pont AUX).
// ---------------------------------------------------------------------------
#define PIN_AUX_L      PB10   // TIM2_CH3 — gate FET bas (PWM de freinage)
#define PIN_AUX_H      PB11   // TIM2_CH4 — gate FET haut (maintenu LOW)

// ============================================================================
//  Motor / power configuration — 26pp hub motor + hall sensors
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
#define CFG_VOLT_LIMIT     23.0f     // motor/driver voltage limit (safety)
#define CFG_VOLT_ALIGN     3.0f     // voltage used during initFOC alignment
#define CFG_CURRENT_LIMIT  5.0f    // A (used once current sensing is enabled)
#define CFG_VEL_LIMIT      100.0f   // rad/s

// ---------------------------------------------------------------------------
//  Gestion de l'énergie régénérée (résistance de freinage 2 ohms sur AUX) +
//  seuils DC bus, pour un bus 24 V nominal. Trois étages, du plus doux au
//  plus dur — ordre requis : BRAKE_ON < BRAKE_FULL <= REGEN_START
//  < REGEN_FULL < OV_TRIP :
//   1. rampe de duty du frein  (BRAKE_ON -> BRAKE_FULL : 0 -> MAX_DUTY)
//   2. dérating du courant de freinage moteur (REGEN_START -> REGEN_FULL)
//   3. faute over-voltage latchée (OV_TRIP) : DRV8301 coupé, frein maintenu
//  Duty max 1.0 = 26.5²/2 ≈ 350 W crête dans la résistance — transitoire ;
//  réduire si la résistance chauffe trop en usage réel.
// ---------------------------------------------------------------------------
#define CFG_BRAKE_R            2.0f    // ohms, résistance sur les bornes AUX
#define CFG_BRAKE_PWM_HZ       20000   // PWM frein (TIM2) — inaudible
#define CFG_BRAKE_MAX_DUTY     1.0f    // 100 % possible : FET bas sans bootstrap

// Rythme d'appel de updateBusSafety() (voir SafetyTask) : la mesure Vbus fait
// une conversion ADC bloquante -- inutile de la faire à 1kHz (rien ne bouge
// aussi vite sur un bus batterie), et ça évite de voler du CPU à FOCTask
// (SafetyTask est plus prioritaire). SafetyTask tourne à 1kHz de base, donc
// CFG_BUS_SAFETY_HZ doit rester un diviseur entier de 1000.
#define CFG_BUS_SAFETY_HZ      200
#define CFG_BUS_SAFETY_DIV     (1000 / CFG_BUS_SAFETY_HZ)
#define CFG_BUS_SAFETY_DT      (1.0f / CFG_BUS_SAFETY_HZ)

// Limite de pente du duty frein (duty/s). Sans rampe, tout saut de Vbus
// (franchissement de BRAKE_ON) ou de courant régénéré (le couple change de
// signe en 1-2 cycles PID) se traduit par un saut de duty quasi instantané --
// à-coup mécanique sur la roue. 6.0 = 0->100% en ~167ms : à resserrer si le
// bus dépasse REGEN_START/OV_TRIP pendant un freinage franc (la rampe retarde
// la réaction), à desserrer si le freinage reste perceptible comme un à-coup.
#define CFG_BRAKE_RAMP         6.0f
#define CFG_VBUS_BRAKE_ON      25.5f   // V — début de la rampe frein
#define CFG_VBUS_BRAKE_FULL    26.5f   // V — frein à MAX_DUTY
#define CFG_VBUS_REGEN_START   26.5f   // V — début dérating couple de freinage
#define CFG_VBUS_REGEN_FULL    28.0f   // V — courant régen totalement coupé
#define CFG_VBUS_OV_TRIP       29.0f   // V — faute latchée (10 ms consécutives)

// Consigne de vitesse max acceptée (rad/s) : ~90 % de la vitesse à vide
// atteignable sous CFG_VOLT_LIMIT (KV en rpm/V -> *0.10472 en (rad/s)/V).
// Au-delà, la consigne est physiquement inatteignable : le PID sature et
// l'intégrateur se charge au max sans jamais converger.
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
#define CFG_LPF_CUR_TF     0.01f   // current measurement low-pass (s)
#define CFG_CHAR_VOLTAGE   1.0f     // voltage used by characteriseMotor() for R/L

// ---------------------------------------------------------------------------
//  Pre-calibration ("saved" motor params). Run the commissioning procedure once
//  (see docs/Calibration.md), copy the printed numbers here, then set
//  CFG_PRECALIBRATED 1 so the board arms WITHOUT any calibration motion — the
//  compile-time equivalent of ODrive's pre_calibrated. (Flash-runtime saving is
//  a later phase.) Leave 0 to auto-align on each first arm.
// ---------------------------------------------------------------------------
#define CFG_PRECALIBRATED    0         // 1 = use the values below, skip alignment
#define CFG_ZERO_ELEC_ANGLE  0.0000f   // motor.zero_electric_angle (rad), from initFOC
#define CFG_SENSOR_DIRECTION 1         // +1 = CW, -1 = CCW, from initFOC
#define CFG_PHASE_R          0.0f      // phase resistance (ohm); 0 = leave unset
#define CFG_PHASE_L          0.0f      // phase inductance (H);   0 = leave unset

// ============================================================================
//  FreeRTOS timing / priorities  (higher number = higher urgency)
// ============================================================================
#define FOC_TICK_HZ        20000    // FOC loop rate (TIM6 -> FOCTask notify)
#define MOTION_DOWNSAMPLE  20       // move() runs at FOC_TICK_HZ/DOWNSAMPLE = 1 kHz

#define PRIO_SAFETY        5        // top: fault latch / watchdog
#define PRIO_FOC           4        // FOC loop
#define PRIO_CAN           3        // CAN RX drain (Phase 6)
#define PRIO_COMMS         PRIO_CAN // alias: CAN + control-bridge task
#define PRIO_TELEMETRY     2        // telemetry / debug

// NVIC preemption priority for any ISR that calls a FreeRTOS *FromISR API.
// STM32duino FreeRTOS: configMAX_SYSCALL_INTERRUPT_PRIORITY derives from
// library value 5, so such ISRs must sit at a NUMERICALLY >= 5 priority
// (i.e. less urgent). 6 gives margin. The current-sense ADC ISR (no FreeRTOS
// call) may stay more urgent.
#define NVIC_PRIO_RTOS_SAFE  6

// Task stack depths (in WORDS = 4 bytes). Kept modest to fit the default
// FreeRTOS heap; bump if xTaskCreate returns pdFAIL.
#define STACK_FOC        768
#define STACK_SAFETY     384   // updateBusSafety : HAL ADC + Serial sur faute
#define STACK_TELEMETRY  512
#define STACK_COMMS      768

// ============================================================================
//  CAN (ODrive CANSimple) — Phase 6
// ============================================================================
#define CFG_CAN_NODE_ID   0
#define CFG_CAN_BAUD      500000     // must match CAN_BAUD in can_utilities/src/main.cpp (500 kbit/s)
#define CFG_WATCHDOG_MS   0          // CAN setpoint timeout; 0 = disabled.
                                     // Set e.g. 250 for an e-bike so that losing
                                     // the CAN master disarms the motor.
#define CFG_VBUS_DIV      19.0f      // Vbus ADC divider ratio — verify against your board

// ============================================================================
//  Motion controller defaults (velocity / position modes over CAN)
// ============================================================================
// En foc_current la sortie du PID vitesse est un courant (A), plus une tension.
// Points de départ à re-tuner sur banc.
#define CFG_VEL_P        0.5f        // A/(rad/s)
#define CFG_VEL_I        0.001f        // A/(rad·s⁻¹·s)
// D=0 : HallSensor::getVelocity() (Simple FOC) dérive du dernier intervalle
// inter-front UNIQUEMENT (pas de moyenne glissante) -> avec 26pp (156
// segments/tour) le moindre écart d'espacement mécanique entre aimants fait
// alterner la vitesse instantanée sur-estimée/sous-estimée d'un front à
// l'autre. Un gain D différencie ce bruit tel quel dans la commande de
// courant -> Iq (et donc le couple) suit le même zigzag -> saccades.
#define CFG_VEL_D        0.0001f
#define CFG_VEL_RAMP     30.0f      // PID output ramp (A/s)
#define CFG_POS_P        1.0f       // position P gain ((rad/s)/rad)
// Redescendu de 0.15s : ce filtre compensait le bruit de mesure hall avant
// que HallSensorSmoothVel ne le corrige à la source (moyennage multi-front,
// voir CFG_HALL_VEL_WINDOW ci-dessous). Un Tf=0.15s ajoute ~150ms de retard
// dans la boucle vitesse en plus de la fenêtre de 20ms -- assez pour ronger
// la marge de phase et transformer le bruit en oscillation entretenue
// (observé sur banc : ~5 Hz, quasi indépendant de la vitesse cible, signature
// classique d'un cycle limite plutôt que d'un bruit de capteur). Redescendu à
// 0.02s : lissage résiduel léger, retard total boucle ~fenêtre+Tf ~40ms.
#define CFG_LPF_VEL_TF   0.02f       // velocity low-pass (s)

// Hall velocity averaging window (s) -- see src/HallSensorSmoothVel.h. Forces
// Sensor::getVelocity() to span multiple hall edges per computation instead
// of one, canceling sector-to-sector mechanical spacing error. At 26pp, edge
// period is ~2*PI/(pole_pairs*6*vel_rad_s) -- e.g. ~8ms at 5 rad/s, so 20ms
// spans ~2-3 edges there and more at higher speed (self-improving). Too large
// = more feedback lag; re-tune if response feels sluggish or too small still
// jerky.
#define CFG_HALL_VEL_WINDOW  0.02f
