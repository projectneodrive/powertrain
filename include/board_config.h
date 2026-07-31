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

#pragma region "Pinout / Hardware Configuration"
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
//  Topologie ODrive v3.6 : résistance entre le point milieu et la MASSE. Le
//  FET HAUT tire le point milieu vers DC+ (c'est lui qui dissipe), le FET BAS
//  le ramène à la masse. Les deux sont pilotés en COMPLÉMENTAIRE avec temps
//  mort, exactement comme le firmware ODrive (safety_critical_apply_brake_
//  resistor_timings) :
//     FET BAS  ON de 0            à period*(1-duty) - dt   (PWM mode 1)
//     FET HAUT ON de period*(1-duty) + dt   à period       (PWM mode 2)
//  Piloter un seul FET ne produit RIEN, et c'est mesuré sur ce banc :
//   - FET BAS seul : le point milieu est tiré à la masse = 0 V aux bornes de
//     la résistance, donc 0 A.
//   - FET HAUT seul : son driver est à bootstrap ; le condensateur de bootstrap
//     ne se charge que quand le FET BAS tire le point milieu vers le bas. Sans
//     commutation du bas, VB-VS reste nul et le FET HAUT ne s'ouvre jamais.
//  D'où l'obligation du complémentaire — et l'obligation d'un temps mort non
//  nul, sous peine de court-circuit franc du bus.
//  /!\ PIÈGE MATÉRIEL : le VDD du LM5109B est câblé sur GVDD, le régulateur de
//  grille INTERNE du DRV8301. GVDD n'existe que si EN_GATE est haut. Le frein
//  ne peut donc commuter QUE lorsque l'étage de puissance est réveillé —
//  EN_GATE bas => demi-pont AUX non alimenté, les grilles ne bougent pas quels
//  que soient les registres du timer.
// ---------------------------------------------------------------------------
#define PIN_AUX_L      PB10   // TIM2_CH3 — gate FET bas  (entrée LI de U7)
#define PIN_AUX_H      PB11   // TIM2_CH4 — gate FET haut (entrée HI, dissipe)
#pragma endregion

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
#define CFG_VOLT_LIMIT     23.5f    // motor/driver voltage limit (safety)
#define CFG_VOLT_ALIGN     2.0f     // voltage used during initFOC alignment
#define CFG_CURRENT_LIMIT  4.0f     // A (used once current sensing is enabled)
#define CFG_VEL_LIMIT      17.78f   // rad/s

// Plafonds de sécurité pour les réglages runtime via série (commandes 'LC'/'LV'
// depuis le GUI de config). Un client distant ne doit JAMAIS pouvoir demander un
// courant/vitesse arbitraire : on borne la valeur acceptée ici.
#define CFG_CURRENT_LIMIT_MAX  20.0f   // A — plafond dur pour 'LC'
#define CFG_VEL_LIMIT_MAX      40.0f   // rad/s — plafond dur pour 'LV'

// ---------------------------------------------------------------------------
//  Gestion de l'énergie régénérée (résistance de freinage 2 ohms sur AUX) +
//  seuils DC bus, pour un bus 24 V nominal. Trois étages, du plus doux au
//  plus dur — ordre requis :
//      BRAKE_VBUS_OFF < BRAKE_VBUS_ON < REGEN_START < REGEN_FULL < OV_TRIP
//   1. chopper sur la résistance (BRAKE_VBUS_ON, gain proportionnel)
//   2. dérating du couple de freinage moteur (REGEN_START -> REGEN_FULL)
//   3. faute over-voltage latchée (OV_TRIP)
//  L'ordre est ce qui fait que la résistance a TOUJOURS sa chance avant qu'on
//  sacrifie du couple de freinage, et avant la faute.
// ---------------------------------------------------------------------------
#define CFG_BRAKE_R            2.0f    // ohms, résistance sur les bornes AUX
#define CFG_BRAKE_PWM_HZ       20000   // PWM frein (TIM2) — inaudible
// Plafond < 1.0 IMPÉRATIF : le driver du FET haut (LM5109B) est à bootstrap,
// son condensateur C70 ne se recharge que pendant la conduction du FET bas.
// 0.7 laisse 15 us de recharge par période à 20 kHz — très large.
// 0.7 * 24²/2 ≈ 200 W crête dans la résistance.
#define CFG_BRAKE_MAX_DUTY     0.7f
// Temps mort entre l'ouverture d'un FET et la fermeture de l'autre. En dessous
// du temps de commutation réel des FETs AUX, les deux conduisent brièvement =
// court-circuit du bus. 500 ns est très conservateur (ODrive utilise ~240 ns).
// En center-aligned il est ménagé sur LES DEUX fronts (voir brake.cpp).
#define CFG_BRAKE_DEADTIME_NS  500

// Rythme d'appel de updateBusSafety() (voir SafetyTask) : la mesure Vbus fait
// une conversion ADC bloquante -- inutile de la faire à 1kHz (rien ne bouge
// aussi vite sur un bus batterie), et ça évite de voler du CPU à FOCTask
// (SafetyTask est plus prioritaire). SafetyTask tourne à 1kHz de base, donc
// CFG_BUS_SAFETY_HZ doit rester un diviseur entier de 1000.
#define CFG_BUS_SAFETY_HZ      200
#define CFG_BUS_SAFETY_DIV     (1000 / CFG_BUS_SAFETY_HZ)

// ---------------------------------------------------------------------------
//  Source d'alimentation du bus. Change UNIQUEMENT les seuils : la source
//  détermine si l'énergie de freinage a quelque part où aller.
//
//   PSU      : une alimentation de labo ne peut PAS absorber de courant. Toute
//              l'énergie de freinage part dans la capacité de bus -> il faut
//              dissiper TOUT DE SUITE. Le frein démarre juste au-dessus de la
//              tension de service, et le dérating de couple reste loin
//              au-dessus : c'est la résistance qui fait le travail, on ne
//              sacrifie du couple qu'en dernier recours.
//   BATTERY  : la batterie absorbe le courant de régénération (elle se
//              recharge) tant qu'elle n'est pas pleine. On laisse donc le bus
//              monter jusqu'à la tension de fin de charge AVANT d'allumer le
//              frein : l'énergie est récupérée d'abord, dissipée ensuite.
//
//  /!\ En mode BATTERY, régler BRAKE_VBUS_ON sur la tension de fin de charge
//      réelle du pack (ex. 6S Li-ion = 4.20 V/cellule = 25.2 V). Trop haut =
//      surcharge des cellules.
// ---------------------------------------------------------------------------
#define CFG_BUS_SOURCE_PSU      1
#define CFG_BUS_SOURCE_BATTERY  2

#ifndef CFG_BUS_SOURCE
#define CFG_BUS_SOURCE  CFG_BUS_SOURCE_PSU
#endif

#if CFG_BUS_SOURCE == CFG_BUS_SOURCE_PSU
  // Bus mesuré en fonctionnement : 23.5 V (accélération) à 24.2 V (pointe).
  // BRAKE_VBUS_ON doit rester au-dessus de 24.2 pour ne pas moduler en usage
  // normal, et aussi bas que possible pour dissiper avant que ça monte.
  #define CFG_BRAKE_VBUS_ON      24.6f   // V — le chopper s'engage
  #define CFG_BRAKE_VBUS_OFF     24.2f   // V — il se désengage (hystérésis)
  // Dérating du couple de freinage : filet de sécurité seulement. Si la
  // résistance suffit, on n'arrive jamais ici.
  #define CFG_VBUS_REGEN_START   26.5f   // V — début dérating couple
  #define CFG_VBUS_REGEN_FULL    27.5f   // V — couple de freinage totalement coupé
#elif CFG_BUS_SOURCE == CFG_BUS_SOURCE_BATTERY
  // 6S Li-ion : 25.2 V pack plein. En dessous, la régénération recharge le
  // pack et le frein doit rester éteint — sinon on brûle l'énergie qu'on
  // pourrait récupérer.
  #define CFG_BRAKE_VBUS_ON      25.8f   // V — pack plein, plus rien à absorber
  #define CFG_BRAKE_VBUS_OFF     25.4f   // V
  #define CFG_VBUS_REGEN_START   27.0f   // V
  #define CFG_VBUS_REGEN_FULL    28.0f   // V
#else
  #error "CFG_BUS_SOURCE doit valoir CFG_BUS_SOURCE_PSU ou CFG_BUS_SOURCE_BATTERY"
#endif

// Gain du chopper (duty par volt au-dessus de BRAKE_VBUS_OFF). 0.1/V -> à 1 V
// de dépassement, duty 0.10 = 0.10 * 24²/2 ≈ 29 W dissipés. Mesuré sur banc :
// la capacité de bus (~1400 uF) monte de 24 à 28.5 V avec seulement ~0.4 W de
// régénération, donc quelques dizaines de W suffisent très largement à tenir
// le bus. Monter le gain si le bus dépasse quand même BRAKE_VBUS_ON + 2 V.
#define CFG_BRAKE_GAIN         0.1f

#define CFG_VBUS_OV_TRIP       29.0f   // V — faute latchée (~10 ms consécutives)

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
//  Mode sensorless (observateur de flux MESC/Lemming) au-dessus d'un seuil de
//  vitesse — supprime le plancher de quantification hall à vitesse. Le hall
//  reste actif en dessous ; bascule fondue sur [VEL_LO, VEL_HI]. Prérequis :
//  current-sense actif + CFG_PHASE_R/L renseignés (voir HybridSensor.h).
//  MISE EN SERVICE : garder ENABLE=0, tourner, observer 'obsdV' en télémétrie
//  (doit rester ~0 sur toute la plage) AVANT de passer ENABLE à 1.
// ---------------------------------------------------------------------------
#define CFG_SENSORLESS_ENABLE  1          // 1 = bascule hall->observateur au-dessus du seuil
#define CFG_SENSORLESS_VEL_LO  5.0f       // rad/s : en dessous = hall pur
#define CFG_SENSORLESS_VEL_HI  7.0f       // rad/s : au-dessus = observateur pur

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
#define STACK_SAFETY     512   // updateBusSafety : HAL ADC + Serial (message de faute)
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
// En foc_current la sortie du PID vitesse est un courant (A). À re-tuner par
// moteur. P trop élevé amplifie le bruit de mesure hall -> Iq oscille et le
// régen des à-coups peut faire fauter le bus (observé dès P=1.0) ; monter par
// petits pas (+0.1).
#define CFG_VEL_P        0.1f        // A/(rad/s)
// I fixe le courant de croisière (Ti=P/I). Trop faible -> stick-slip au
// décollage (le rotor colle puis décroche). Monter par paliers si le bas régime
// accroche, baisser si ça dépasse/oscille en régime établi.
#define CFG_VEL_I        0.05f       // A/(rad·s⁻¹·s)
// D dérive le bruit hall directement dans Iq -> garder très bas (le lissage
// multi-front de HallSensorSmoothVel traite déjà ce bruit à la source).
#define CFG_VEL_D        0.0001f
// Pente max du COURANT de sortie du PID (A/s). Une rampe large accélère aussi
// le renversement de couple en freinage -> pic de tension bus plus rapide.
#define CFG_VEL_RAMP     30.0f      // PID output ramp (A/s)
// Limite d'accélération de la CONSIGNE de vitesse (rad/s²) — distincte de
// CFG_VEL_RAMP (courant) : lisse la cible. Sans rampe, un échelon (ex. V5->V10)
// fait plonger puis overshooter l'alim de banc (regen -> OV_TRIP). Trop rapide,
// la rampe qui s'arrête net à la cible excite aussi un dépassement de vitesse en
// arrivée (ring ~5 Hz) qui régénère et fait clignoter le frein = saccade.
// 10 = 0->10 rad/s en ~1 s. Baisser (6-8) si l'arrivée reste saccadée, monter
// pour une réponse plus vive. 0 = échelon direct.
#define CFG_VEL_ACCEL    10.0f      // rad/s²  (0 = pas de rampe de consigne)
#define CFG_POS_P        1.0f       // position P gain ((rad/s)/rad)
#define CFG_POS_I        0.0f       // position I gain (tunable via PI; usually 0)
#define CFG_POS_D        0.0f       // position D gain (tunable via PD; usually 0)
// Passe-bas sur la vitesse mesurée (s). Trop grand ajoute du retard de boucle
// -> oscillation entretenue (cycle limite ~5 Hz vu à 0.15s). Le lissage
// principal est fait par CFG_HALL_VEL_WINDOW ; ce filtre reste léger.
#define CFG_LPF_VEL_TF   0.02f       // velocity low-pass (s)

// Fenêtre de moyennage de la vitesse hall (s) -- voir src/HallSensorSmoothVel.h.
// Force Sensor::getVelocity() à couvrir plusieurs fronts hall par calcul (annule
// l'erreur d'espacement secteur à secteur). Période inter-front ~2π/(pp*6*vel) :
// ~20ms à 2 rad/s, ~8ms à 5 rad/s ; 0.05s couvre ~2.5 fronts à 2 rad/s. Trop
// grand = retard de boucle, trop petit = quantification (saccades à bas régime).
#define CFG_HALL_VEL_WINDOW  0.05f

// ---------------------------------------------------------------------------
//  Calibration des angles de transition hall (anti-ondulation de commutation).
//  SimpleFOC suppose des secteurs hall de 60° élec. pile ; le placement réel est
//  irrégulier -> angle de commutation faux dans chaque secteur -> ondulation de
//  couple à toutes les vitesses. Voir src/HallSensorSmoothVel.h.
//  Procédure : moteur DÉSARMÉ -> commande série 'H' (spin boucle ouverte ~10s)
//  -> copier les 6 offsets imprimés dans CFG_HALL_CAL_OFFSETS -> passer
//  CFG_HALL_PRECALIBRATED à 1 -> rebuild. Sinon les offsets ne vivent qu'en RAM.
// ---------------------------------------------------------------------------
#define CFG_HALL_CAL_VOLTAGE     2.0f    // V, tension du spin boucle ouverte
#define CFG_HALL_CAL_ELEC_SPEED  8.0f    // rad/s élec (~0.31 rad/s méca à 26pp)
#define CFG_HALL_CAL_REVS        12      // tours élec. balayés (2 premiers ignorés)
#define CFG_HALL_PRECALIBRATED   1       // 1 = charger CFG_HALL_CAL_OFFSETS au boot
// Offsets mécaniques (rad) par secteur hall 0..5, produits par 'H' (direction
// incluse). Secteurs 0..5 en deg élec : +0.92 -1.18 +0.46 +0.36 -1.33 +0.77.
#define CFG_HALL_CAL_OFFSETS  { -0.0006175f, 0.0007926f, -0.0003101f, -0.0002395f, 0.0008929f, -0.0005184f }
