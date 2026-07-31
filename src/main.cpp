// ============================================================================
//  SimpleFOC + FreeRTOS + ODrive CANSimple  —  ODrive v3.6 (MKS clone) / F405
// ============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <STM32FreeRTOS.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/smoothing/SmoothingSensor.h"
#include "HallSensorSmoothVel.h"
#include "HybridSensor.h"
#include "current_sense/hardware_specific/stm32/stm32_mcu.h"  // Stm32CurrentSenseParams
#include "drv8301.h"
#include "odrive_can.h"
#include "board_config.h"
#include "brake.h"

using namespace odcan;

// ============================================================================
//  ODRIVE CLOCK CONFIG (8 MHz HSE -> 168 MHz)
// ============================================================================
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while (1); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { while (1); }
  SystemCoreClockUpdate();
}

// ============================================================================
//  Objects & Hardware Association
// ============================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_M0_INH_A, PIN_M0_INL_A,
                                       PIN_M0_INH_B, PIN_M0_INL_B,
                                       PIN_M0_INH_C, PIN_M0_INL_C, PIN_EN_GATE);
BLDCMotor motor = BLDCMotor(CFG_POLE_PAIRS);

#if SENSOR_TYPE == SENSOR_TYPE_HALL
HallSensorSmoothVel sensor = HallSensorSmoothVel(PIN_ENC_A, PIN_ENC_B, PIN_ENC_Z, CFG_POLE_PAIRS);
static void doHallA() { sensor.handleA(); }
static void doHallB() { sensor.handleB(); }
static void doHallC() { sensor.handleC(); }
// Interpole l'angle entre deux fronts hall (60° elec. de résolution sinon)
SmoothingSensor smooth = SmoothingSensor(sensor, motor);
// Hybride hall + observateur sensorless (bascule à vitesse). Transparent (=hall
// pur) tant que hybrid.enabled est faux. Voir HybridSensor.h.
HybridSensor hybrid = HybridSensor(smooth, motor);
Sensor& foc_sensor = hybrid;
#else
STM32HWEncoder sensor = STM32HWEncoder(CFG_ENC_PPR, PIN_ENC_A, PIN_ENC_B);
Sensor& foc_sensor = sensor;
#endif

SPIClass spi3(PIN_DRV_MOSI, PIN_DRV_MISO, PIN_DRV_SCK);
DRV8301 drv(spi3, PIN_M0_CS);
LowsideCurrentSense current_sense = LowsideCurrentSense(CFG_SHUNT_OHMS, CFG_DRV_GAIN, _NC, PIN_M0_IB, PIN_M0_IC);

AxisIO   g_io;        // Shared command/telemetry block
OdriveCAN g_can(g_io); // CANSimple instance

// ============================================================================
//  Shared State & RTOS Handles
// ============================================================================
volatile float g_active_target = 0.0f;
volatile bool  g_fault         = false;
volatile bool  g_focReady      = false;
static   bool  g_calibrated    = false;
static   volatile bool g_req_hall_cal = false;  // commande série 'H' -> calib angles hall
static   bool  g_iSenseOk      = false;
static   TaskHandle_t g_focTask = nullptr;
static   HardwareTimer *g_focTimer = nullptr;

// --- Télémétrie capteur (écrite par FOCTask SEULEMENT, lue par CommsTask).
//     getAngle() lit full_rotations + angle_prev : paire non atomique, mise à
//     jour à 20 kHz par FOCTask. La lire depuis CommsTask donne des lectures
//     déchirées = spikes de ±1 tour dans pos_rev. Un float volatile écrit par
//     un seul writer est, lui, atomique sur Cortex-M4. ---
volatile float g_shaft_angle = 0.0f;   // rad, multi-tours (convention capteur)
volatile float g_shaft_vel   = 0.0f;   // rad/s

// --- DC bus safety (écrit par SafetyTask, lu par FOCTask/CommsTask) ---
volatile float g_vbus_filt      = 0.0f;              // Vbus filtré (V)
volatile float g_regen_iq_limit = CFG_CURRENT_LIMIT; // |Iq| de freinage max (A)

// 20 kHz FOC tick ISR
static void onFocTick() {
  BaseType_t hpw = pdFALSE;
  if (g_focTask) vTaskNotifyGiveFromISR(g_focTask, &hpw);
  portYIELD_FROM_ISR(hpw);
}

// ============================================================================
//  Logic Helpers
// ============================================================================
// ---------------------------------------------------------------------------
//  DC bus safety : mesure Vbus + résistance de freinage + dérating régen.
//  Tout tourne dans SafetyTask (1 kHz), SEULE tâche à toucher l'ADC Vbus —
//  publishTelemetry lit g_vbus_filt (pas de conversion concurrente).
// ---------------------------------------------------------------------------

// Vbus sur un ADC DÉDIÉ, jamais celui des shunts. Historique des échecs sur
// banc (bus réel 24 V) :
//  - analogRead() : STM32duino DeInit l'ADC -> tue les conversions injectées.
//  - _readRegularADCVoltage() (canal régulier SUR l'ADC des shunts) : le
//    trigger injecté (TIM1, 20 kHz) avorte/reprend la conversion régulière en
//    plein échantillonnage, qui repart avec la tension du canal injecté dans
//    le S/H -> ~5.4 V à l'arrêt (amplis éteints, 0 V), ~30 V moteur armé
//    (amplis à ~1.65 V). Fausse faute OV systématique pendant l'alignement,
//    même avec 480 cycles d'échantillonnage, médiane et anti-rebond 10 ms.
//  PA6 = ADC12_IN6 : on prend celui d'ADC1/ADC2 que le current sense
//  n'occupe pas -> aucune interaction possible avec les injectées.
static ADC_HandleTypeDef g_vbusAdc = {};

static bool vbusAdcInit() {
  ADC_TypeDef *cs_inst = nullptr;
  if (g_iSenseOk && current_sense.params) {
    ADC_HandleTypeDef *h = ((Stm32CurrentSenseParams *)current_sense.params)->adc_handle;
    if (h) cs_inst = h->Instance;
  }
  ADC_TypeDef *inst = (cs_inst == ADC1) ? ADC2 : ADC1;

  if (inst == ADC1) { __HAL_RCC_ADC1_CLK_ENABLE(); }
  else              { __HAL_RCC_ADC2_CLK_ENABLE(); }
  pinmap_pinout(digitalPinToPinName(PIN_VBUS), PinMap_ADC);  // PA6 analogique

  g_vbusAdc.Instance = inst;
  // Même prescaler que la lib (registre COMMUN aux deux ADC — ne pas dévier).
  g_vbusAdc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  g_vbusAdc.Init.Resolution            = ADC_RESOLUTION_12B;
  g_vbusAdc.Init.ScanConvMode          = DISABLE;
  g_vbusAdc.Init.ContinuousConvMode    = DISABLE;
  g_vbusAdc.Init.DiscontinuousConvMode = DISABLE;
  g_vbusAdc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  g_vbusAdc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  g_vbusAdc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  g_vbusAdc.Init.NbrOfConversion       = 1;
  g_vbusAdc.Init.DMAContinuousRequests = DISABLE;
  g_vbusAdc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&g_vbusAdc) != HAL_OK) { g_vbusAdc.Instance = nullptr; return false; }

  ADC_ChannelConfTypeDef c = {};
  c.Channel      = ADC_CHANNEL_6;             // PA6 = ADC12_IN6
  c.Rank         = 1;
  c.SamplingTime = ADC_SAMPLETIME_480CYCLES;  // diviseur haute impédance
  c.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&g_vbusAdc, &c) != HAL_OK) { g_vbusAdc.Instance = nullptr; return false; }

  Serial.print("Vbus ADC: dedicated ADC");
  Serial.print(inst == ADC1 ? 1 : 2);
  Serial.print(" (current sense on ADC");
  Serial.print(cs_inst == ADC1 ? "1" : (cs_inst == ADC2 ? "2" : "?"));
  Serial.println(")");
  return true;
}

// Conversion one-shot bloquante (~24 µs) — appelée uniquement par SafetyTask.
static float readVbusRaw() {
  if (!g_vbusAdc.Instance) return -1.0f;
  if (HAL_ADC_Start(&g_vbusAdc) != HAL_OK) return -1.0f;
  float v = -1.0f;
  if (HAL_ADC_PollForConversion(&g_vbusAdc, 1) == HAL_OK)
    v = HAL_ADC_GetValue(&g_vbusAdc) * (3.3f / 4096.0f);
  HAL_ADC_Stop(&g_vbusAdc);
  return v;
}

// Médiane de 3 : rejette tout échantillon isolé aberrant (conversion avortée
// par un trigger injecté, transitoire de commutation) avant le LPF.
static float median3(float a, float b, float c) {
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

static void updateBusSafety() {
  static float s0 = -1.0f, s1 = -1.0f;          // 2 derniers échantillons bruts
  float v = readVbusRaw();                      // volts au pin; -1.0f si erreur
  if (v >= 0.0f) {
    v *= CFG_VBUS_DIV;
    if (s0 < 0.0f) { s0 = v; s1 = v; }          // amorçage
    float m = median3(s0, s1, v);
    s0 = s1; s1 = v;
    float f = g_vbus_filt;                      // LPF 1er ordre, tau ≈ 2 ms
    g_vbus_filt = (f <= 0.0f) ? m : f + 0.33f * (m - f);
  }
  float vb = g_vbus_filt;
  if (vb <= 0.0f) { brake::off(); return; }        // pas encore de mesure

  // Étage 3 : faute over-voltage latchée. Anti-rebond ~10 ms consécutives :
  // un transitoire ou des échantillons corrompus ne doivent JAMAIS latcher.
  // updateBusSafety() tourne maintenant à 200 Hz (5 ms/appel, voir
  // SafetyTask) au lieu de 1 kHz -- 2 appels consécutifs plutôt que 10 pour
  // garder le même temps de réponse réel (~10 ms).
  //
  // On arrive ici seulement si les étages 1 (dissipation) et 2 (dérating du
  // couple) n'ont pas suffi : la résistance a toujours eu sa chance avant,
  // c'est garanti par l'ordre des seuils (voir board_config.h).
  // Couper EN_GATE coupe aussi GVDD, donc le frein (VDD du LM5109B vient du
  // DRV8301) : au-delà de OV_TRIP il n'y a plus de dissipation, le moteur est
  // simplement mis en roue libre. C'est le comportement voulu — la faute est
  // le dernier recours, pas un mode de régulation.
  // TODO: pour garder la dissipation active en faute, laisser EN_GATE haut et
  // couper le moteur via BDTR.MOE de TIM1 (six grilles en Hi-Z, GVDD conservée).
  static uint8_t ov_count = 0;
  if (vb > CFG_VBUS_OV_TRIP) {
    if (ov_count < 255) ov_count++;
    if (ov_count >= 2 && !g_fault) {
      digitalWrite(PIN_EN_GATE, LOW);
      g_fault = true;
      g_io.axis_error |= ERR_DC_BUS_OVER_VOLTAGE;
      Serial.print("[FAULT] DC bus over-voltage: ");
      Serial.print(vb, 1); Serial.println(" V");
    }
  } else {
    ov_count = 0;
  }

  // Étage 2 : dérating du courant de freinage autorisé (consommé par FOCTask).
  // Filet de sécurité seulement : les seuils REGEN_* sont au-dessus de ceux du
  // chopper, donc on ne sacrifie du couple de freinage que si la dissipation
  // n'a pas suffi à tenir le bus.
  float s = 1.0f - (vb - CFG_VBUS_REGEN_START)
                 / (CFG_VBUS_REGEN_FULL - CFG_VBUS_REGEN_START);
  g_regen_iq_limit = motor.current_limit * _constrain(s, 0.0f, 1.0f);

  // Étage 1 : le chopper est piloté par brake::update() depuis SafetyTask
  // (1 kHz), pas ici — voir SafetyTask.
}

static void enableStage() {
  motor.enable();
  delay(5); // Setup time for DRV8301 SPI stability
  drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
}

#if SENSOR_TYPE == SENSOR_TYPE_HALL
// ---------------------------------------------------------------------------
//  Calibration des angles réels de transition hall (voir board_config.h +
//  HallSensorSmoothVel.h). Spin boucle ouverte à vitesse électrique imposée :
//  l'angle commandé (theta) EST l'angle vrai. Le résidu moyen par secteur entre
//  theta et l'angle rapporté (grille 60° uniforme) = l'erreur de placement du
//  secteur ; on retire la moyenne (offset global -> zero_electric_angle) et on
//  convertit élec->méca. 'dir' (sens secteur vs theta) est mesuré via la
//  séquence des secteurs, donc indépendant du sensor_direction d'initFOC.
//  Bloque CommsTask ~10 s (comme characteriseMotor). À lancer moteur DÉSARMÉ.
// ---------------------------------------------------------------------------
static bool calibrateHallAngles() {
  const float Uq     = CFG_HALL_CAL_VOLTAGE;
  const float wel    = CFG_HALL_CAL_ELEC_SPEED;   // rad/s élec
  const int   revs   = CFG_HALL_CAL_REVS;
  const float dt     = 0.0005f;                    // 500 us / pas
  const float dtheta = wel * dt;
  const float pp     = (float)CFG_POLE_PAIRS;

  Serial.println("Hall cal: spin boucle ouverte ~10s (moteur doit tourner lentement)...");
  enableStage();

  // Verrouiller le rotor sur theta=0 et laisser le transitoire s'amortir, sinon
  // les premières transitions capturées seraient faussées par l'oscillation.
  for (int i = 0; i < 500; i++) { motor.setPhaseVoltage(Uq, 0.0f, 0.0f); delay(1); }

  float sacc[6] = {0}, cacc[6] = {0};   // moyenne circulaire du résidu par secteur
  int   cnt[6]  = {0};
  int   seq_up = 0, seq_down = 0;       // vote sur le sens (secteur vs theta)
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
      int d = s - prev;                                // sens de progression des secteurs
      if (d == 1 || d == -5)      seq_up++;
      else if (d == -1 || d == 5) seq_down++;
      prev = s;
    }
    if (theta < 2.0f * _2PI) continue;                 // 2 tours de mise en régime
    if (dir == 0) dir = (seq_up >= seq_down) ? 1 : -1; // sens figé après le régime

    // Résidu élec = theta - dir*pp*angle_prev, ramené mod 2PI. Le terme
    // full_rotations disparaît mod 2PI (pp*2PI ≡ 0), donc lire angle_prev
    // (getMechanicalAngle, float atomique) suffit et évite tout déchirement.
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

  // Résidu moyen (circulaire) par secteur, puis retrait de la moyenne globale.
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
    float off_e = res[s] - mean;                 // résidu élec mean-zéro
    while (off_e >  _PI) off_e -= _2PI;
    while (off_e < -_PI) off_e += _2PI;
    // Δméca à ajouter à angle_prev : dir*off_e/pp (voir dérivation board_config.h).
    float off_m = (float)dir * off_e / pp;
    sensor.sector_offset[s] = off_m;
    Serial.print("  "); Serial.print(off_m, 7); Serial.print("f,");
    Serial.print("  // secteur "); Serial.print(s);
    Serial.print("  ("); Serial.print(off_e * 180.0f / _PI, 2); Serial.println(" deg elec)");
  }
  Serial.println("};");
  Serial.println("-> copier dans board_config.h puis CFG_HALL_PRECALIBRATED=1 pour figer.");

  sensor.offsets_active = true;
  g_calibrated = false;   // forcer un re-initFOC au prochain arm (zero_electric_angle avec offsets actifs)
  return true;
}
#endif // SENSOR_TYPE_HALL

// Axis State Machine (1 kHz)
static void applyControl() {
  uint32_t now = millis();

  if (g_io.req_reboot) {
    brake::off();
    motor.disable(); digitalWrite(PIN_EN_GATE, LOW);
    NVIC_SystemReset();
  }
  
  if (g_io.req_clear_errors) {
    g_io.req_clear_errors = false;
    g_fault = false;
    g_io.axis_error = 0;
    Serial.println("[OK] Erreurs réinitialisées.");
  }
  
  // --- Gains PID vitesse (CAN Set_Vel_Gains ou série KP/KI/KD) ---
  if (g_io.req_vel_gains) {
    g_io.req_vel_gains = false;
    // g_io.* en Nm/(rad/s) ; en foc_current le PID sort des ampères -> /Kt.
    // En fallback voltage la valeur est appliquée telle quelle (volts).
    float k = g_iSenseOk ? (1.0f / CFG_KT) : 1.0f;
    motor.PID_velocity.P = g_io.vel_gain     * k;
    motor.PID_velocity.I = g_io.vel_int_gain * k;
    motor.PID_velocity.D = g_io.vel_d_gain   * k;
    Serial.print("[PID vel] P="); Serial.print(motor.PID_velocity.P, 4);
    Serial.print(" I=");          Serial.print(motor.PID_velocity.I, 4);
    Serial.print(" D=");          Serial.println(motor.PID_velocity.D, 5);
  }

  // --- Gains PID courant (série JP/JI/JD) : appliqués tels quels aux deux axes
  // (q et d) du régulateur de courant SimpleFOC. En V/A, pas de conversion Kt.
  if (g_io.req_cur_gains) {
    g_io.req_cur_gains = false;
    motor.PID_current_q.P = g_io.cur_p_gain; motor.PID_current_d.P = g_io.cur_p_gain;
    motor.PID_current_q.I = g_io.cur_int_gain; motor.PID_current_d.I = g_io.cur_int_gain;
    motor.PID_current_q.D = g_io.cur_d_gain; motor.PID_current_d.D = g_io.cur_d_gain;
    Serial.print("[PID cur] P="); Serial.print(motor.PID_current_q.P, 4);
    Serial.print(" I=");          Serial.print(motor.PID_current_q.I, 4);
    Serial.print(" D=");          Serial.println(motor.PID_current_q.D, 5);
  }

  // --- Gains I/D de position (série PI/PD). Le P (P_angle.P) est appliqué à
  // chaque tour dans applyControl ; ici on ne pose que I et D (rarement utiles).
  if (g_io.req_pos_gains) {
    g_io.req_pos_gains = false;
    motor.P_angle.I = g_io.pos_int_gain;
    motor.P_angle.D = g_io.pos_d_gain;
    Serial.print("[PID pos] P="); Serial.print(motor.P_angle.P, 4);
    Serial.print(" I=");          Serial.print(motor.P_angle.I, 4);
    Serial.print(" D=");          Serial.println(motor.P_angle.D, 5);
  }

  // Sécurité globale simplifiée (la SafetyTask gère le hardware)
  bool safe = !g_io.estop && !g_fault;

  // --- MOTOR_CALIBRATION (R/L Measurement) ---
  // Toujours répondre (et toujours consommer le flag) : characteriseMotor prend
  // la main sur setPhaseVoltage, donc exige le moteur DÉSARMÉ. Sans ce retour,
  // un 'M' envoyé armé était silencieusement ignoré, flag laissé actif.
  if (g_io.req_characterise) {
    g_io.req_characterise = false;
    if (g_focReady || g_io.armed) {
      Serial.println("[!] M: désarmer d'abord (envoyer 'I'), puis 'M'.");
    } else if (!g_iSenseOk) {
      Serial.println("[!] M: current-sense non initialisé (voir boot).");
    } else if (!safe) {
      Serial.println("[!] M: faute active -> 'C' pour l'effacer d'abord.");
    } else {
      Serial.println("Characterising motor (R/L)... (quelques secondes, moteur immobile)");
      enableStage();
      int rc = motor.characteriseMotor(CFG_CHAR_VOLTAGE);
      motor.disable();
      if (rc == 0) {
        Serial.print("  R = "); Serial.print(motor.phase_resistance, 4);
        Serial.print(" ohm   L = "); Serial.print(motor.phase_inductance * 1e6f, 2);
        Serial.println(" uH");
      } else {
        // 1=CS non init, 2=volt<=0, 3=courant trop faible (monter CFG_CHAR_VOLTAGE), 4=R<=0
        Serial.print("[!] Characterise échec, code "); Serial.println(rc);
      }
    }
    return;
  }

#if SENSOR_TYPE == SENSOR_TYPE_HALL
  // --- CALIBRATION ANGLES HALL (commande série 'H', moteur désarmé) ---
  if (g_req_hall_cal && !g_focReady) {
    g_req_hall_cal = false;
    if (safe && !g_io.armed) {
      calibrateHallAngles();
      motor.disable();
    } else {
      Serial.println("[!] Hall cal exige moteur désarmé + état sain (envoyer 'I' puis 'H').");
    }
    return;
  }
#endif

  bool want = g_io.armed && safe;

  // --- DISARM ---
  if (!want && g_focReady) {
    g_focReady = false;
    g_active_target = 0.0f;
    motor.disable();
  }

  // --- ARM & CALIBRATE ---
  if (want && !g_focReady) {
    enableStage();
    if (!g_calibrated) {
      if (CFG_PRECALIBRATED) {
        motor.sensor_direction = (CFG_SENSOR_DIRECTION >= 0) ? Direction::CW : Direction::CCW;
        motor.zero_electric_angle = CFG_ZERO_ELEC_ANGLE;
      }
      if (motor.initFOC()) {
        g_calibrated = true;
        Serial.print("initFOC OK | CFG_SENSOR_DIRECTION=");
        Serial.print(motor.sensor_direction == Direction::CW ? 1 : -1);
        Serial.print("  CFG_ZERO_ELEC_ANGLE=");
        Serial.println(motor.zero_electric_angle, 4);
      } else {
        Serial.println("[-] initFOC FAILED -> disarm (voir logs MOT: ci-dessus)");
        g_io.axis_error |= ERR_ENCODER_FAILED;
        g_io.armed = false;
        motor.disable();
        return;
      }
    }
    g_focReady = true;
  }

  // --- Runtime Control Modes & Limits ---
  if (g_focReady) {
    switch (g_io.control_mode) {
      case CTRL_VELOCITY: {
        motor.controller = MotionControlType::velocity;
        // Consigne bornée à la vitesse atteignable : au-delà le PID sature
        // et l'intégrateur se charge au max sans jamais converger.
        float vmax = (motor.velocity_limit < CFG_VEL_CMD_MAX) ? motor.velocity_limit
                                                              : CFG_VEL_CMD_MAX;
        float cmd = _constrain(g_io.input_vel, -vmax, vmax);
        // Rampe d'accélération de la consigne (CFG_VEL_ACCEL, rad/s²) : glisse
        // g_active_target vers cmd au lieu d'un échelon. applyControl tourne à
        // 1 kHz -> pas = accel * 1ms. Voir board_config.h (évite le trip OV sur
        // gros échelon avec l'alim de banc). 0 = échelon direct.
        if (CFG_VEL_ACCEL > 0.0f) {
          float step = CFG_VEL_ACCEL * 0.001f;
          g_active_target += _constrain(cmd - g_active_target, -step, step);
        } else {
          g_active_target = cmd;
        }
        break;
      }
      case CTRL_POSITION:
        motor.controller = MotionControlType::angle;
        g_active_target  = g_io.input_pos;
        break;
      case CTRL_TORQUE:
      case CTRL_VOLTAGE:
      default:
        motor.controller = MotionControlType::torque;
        g_active_target = g_iSenseOk ? (g_io.input_torque / CFG_KT) : g_io.input_torque;
        break;
    }
    // Setters SimpleFOC : propagent aussi les limites aux PID internes
    // (en foc_current, PID_velocity.limit = current_limit)
    if (g_io.vel_limit     > 0.0f) motor.updateVelocityLimit(g_io.vel_limit);
    if (g_io.current_limit > 0.0f) motor.updateCurrentLimit(g_io.current_limit);
    if (g_io.pos_gain      > 0.0f) motor.P_angle.P = g_io.pos_gain;

    if (CFG_WATCHDOG_MS > 0 && (now - g_io.last_setpoint_ms) > CFG_WATCHDOG_MS) {
      g_io.axis_error |= ERR_WATCHDOG_EXPIRED;
      g_io.armed = false;
    }
  } else {
    g_active_target = 0.0f;
  }
}

static void publishTelemetry() {
  // Pos/vel publiées par FOCTask (single-writer) : lecture atomique ici.
  float sgn = (motor.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;
  g_io.pos_rev = sgn * g_shaft_angle / TWO_PI;
  g_io.vel_rev = sgn * g_shaft_vel / TWO_PI;
  g_io.vbus    = g_vbus_filt;   // échantillonné/filtré par SafetyTask (1 kHz)
  if (g_focReady && g_iSenseOk) {
    g_io.iq_setpoint = motor.current_sp;
    g_io.iq_measured = motor.current.q;
    float p = motor.voltage.q * motor.current.q + motor.voltage.d * motor.current.d;
    g_io.ibus = (g_io.vbus > 1.0f) ? (p / g_io.vbus) : 0.0f;
  } else {
    g_io.iq_setpoint = 0.0f;
    g_io.iq_measured = 0.0f;
    g_io.ibus        = 0.0f;
  }
  g_io.cur_state = g_focReady ? AXIS_CLOSED_LOOP : AXIS_IDLE;
}
// ============================================================================
//  RTOS Tasks Implementation
// ============================================================================

// FOCTask (20 kHz hard realtime)
static void FOCTask(void *) {
  g_focTask = xTaskGetCurrentTaskHandle();
  g_focTimer = new HardwareTimer(TIM6);
  g_focTimer->setOverflow(FOC_TICK_HZ, HERTZ_FORMAT);
  g_focTimer->attachInterrupt(onFocTick);
  g_focTimer->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);
  g_focTimer->resume();

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Ne JAMAIS écraser motor.shaft_angle/shaft_velocity ici : move() y stocke
    // l'angle multi-tours filtré ; l'écraser mélange deux référentiels dans la
    // télémétrie. Au repos on rafraîchit juste le capteur pour pos/vel.
    if (!g_focReady || g_fault) {
      foc_sensor.update();
    } else {
      motor.loopFOC();             // fait sensor.update() en interne
      motor.move(g_active_target);

      // Dérating régen : ne borne QUE le couple qui s'oppose à la rotation
      // (freinage, sp·vel < 0) — le couple moteur n'est jamais réduit.
      // current_sp persiste entre deux move() (MOTION_DOWNSAMPLE), donc le
      // clamp après move() couvre chaque loopFOC() suivant. En fallback
      // voltage, current_sp est en volts : le clamp reste homogène puisque
      // current_limit y borne aussi des volts.
      float sp = motor.current_sp;
      if (sp * motor.shaft_velocity < 0.0f) {
        float lim = g_regen_iq_limit;
        motor.current_sp = _constrain(sp, -lim, lim);
      }
    }

    // Publication pos/vel à 1 kHz : seul FOCTask touche le capteur, la
    // télémétrie ne lit plus que des floats atomiques.
    static uint8_t tel_cnt = 0;
    if (++tel_cnt >= 20) {
      tel_cnt = 0;
      g_shaft_angle = foc_sensor.getAngle();
      g_shaft_vel   = foc_sensor.getVelocity();
    }
  }
}

// SafetyTask (1 kHz - Hardware fault handler)
static void SafetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t fault_counter = 0;
  uint8_t  vbus_cnt = 0;

  for (;;) {
    // updateBusSafety() fait une conversion ADC BLOQUANTE (~24us, readVbusRaw).
    // SafetyTask tourne à PRIO_SAFETY (5) > PRIO_FOC (4) : si on l'appelle à
    // chaque tick 1kHz, cette tâche plus prioritaire vole ~24us à FOCTask
    // toutes les 1ms pile, sur un budget de 50us/tick à 20kHz -- assez pour
    // faire sauter un tick FOC de temps en temps, systématiquement au rythme
    // de SafetyTask (les notifications RTOS ne sont pas mises en file :
    // ulTaskNotifyTake(pdTRUE,...) efface le compteur, un tick manqué est
    // perdu, pas rattrapé). Observé sur banc : apparition d'une oscillation
    // de vitesse ~5Hz dès l'ajout de ce frein régénératif. Vbus/frein/régen
    // n'ont physiquement pas besoin de 1kHz -- 200Hz suffit très largement
    // (aucune dynamique utile aussi rapide sur un bus batterie) et réduit
    // d'autant l'empiètement sur FOCTask. Le nFAULT digitalRead(), lui,
    // reste à 1kHz : quasi gratuit, et c'est la détection de faute rapide.
    if (++vbus_cnt >= CFG_BUS_SAFETY_DIV) {
      vbus_cnt = 0;
      updateBusSafety();
    }

    // Vérification de nFAULT uniquement si le driver est censé être actif
    bool en_gate = (digitalRead(PIN_EN_GATE) == HIGH);
    if (en_gate && digitalRead(PIN_N_FAULT) == LOW) {
      fault_counter++;
      if (fault_counter > 10) { // Anti-rebond : 10 ms consécutives à bas
        digitalWrite(PIN_EN_GATE, LOW); // Coupure d'urgence immédiate
        g_fault = true;
        g_io.axis_error |= ERR_MOTOR_FAILED;
        en_gate = false;
      }
    } else {
      fault_counter = 0; // Reset si le signal revient au vert ou driver éteint
    }

    // Chopper de dissipation. Appelé à 1 kHz alors que Vbus n'est rafraîchi
    // qu'à 200 Hz : ce n'est pas la régulation qu'on accélère, c'est la
    // COUPURE. Un désarmement/une faute est pris en compte en 1 ms au lieu de
    // 5, pour deux écritures registre — l'ADC bloquant, lui, reste à 200 Hz
    // (voir le commentaire de perf ci-dessus).
    // Le frein n'est physiquement alimenté que si EN_GATE est haut (GVDD vient
    // du DRV8301) : la condition n'est donc pas une précaution, c'est la
    // réalité du câblage.
    brake::update(g_vbus_filt, g_focReady && !g_fault && en_gate);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

// CommsTask (1 kHz - CAN Simple + Machine d'état)
static void CommsTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    g_can.poll();
    applyControl();
    publishTelemetry();
    g_can.txCyclic(millis());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

// Dump de la configuration courante sur une seule ligne préfixée "cfg " pour
// que le GUI la distingue de la télémétrie ("t=..."). Les champs de limites et
// de gains sont réglables via série (LC/LV ; pos PP/PI/PD ; vitesse KP/KI/KD ;
// courant JP/JI/JD) ; les suivants sont des constantes matérielles compile-time,
// en lecture seule côté GUI.
static void reportConfig() {
  Serial.print("cfg current_limit="); Serial.print(motor.current_limit, 3);
  Serial.print(" vel_limit=");        Serial.print(motor.velocity_limit, 3);
  Serial.print(" pos_gain=");         Serial.print(motor.P_angle.P, 4);
  Serial.print(" pos_i=");            Serial.print(motor.P_angle.I, 4);
  Serial.print(" pos_d=");            Serial.print(motor.P_angle.D, 5);
  Serial.print(" vel_p=");            Serial.print(g_io.vel_gain, 4);
  Serial.print(" vel_i=");            Serial.print(g_io.vel_int_gain, 4);
  Serial.print(" vel_d=");            Serial.print(g_io.vel_d_gain, 5);
  Serial.print(" cur_p=");            Serial.print(motor.PID_current_q.P, 4);
  Serial.print(" cur_i=");            Serial.print(motor.PID_current_q.I, 4);
  Serial.print(" cur_d=");            Serial.print(motor.PID_current_q.D, 5);
  Serial.print(" pole_pairs=");       Serial.print(CFG_POLE_PAIRS);
  Serial.print(" kv=");               Serial.print(CFG_KV, 2);
  Serial.print(" kt=");               Serial.print(CFG_KT, 4);
  Serial.print(" phase_r=");          Serial.print(motor.phase_resistance, 4);
  Serial.print(" phase_l=");          Serial.print(motor.phase_inductance * 1e6f, 2);
  Serial.print(" vbus_nom=");         Serial.print(CFG_VBUS_NOMINAL, 1);
  Serial.print(" volt_limit=");       Serial.print(motor.voltage_limit, 1);
  Serial.println();
}

// SerialTask (10 Hz - Debug Console)
static void handleSerial() {
  static char buf[24];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf[idx] = '\0';
      if (idx > 0) {
        float v = atof(buf + 1);
        // Acknowledge (AK) : réponse synchrone, sans timestamp, montrant
        // l'ancienne -> la nouvelle valeur (ou l'accusé de réception).
        switch (buf[0]) {
          case 'A': case 'a': {
            bool old = g_io.armed;
            g_io.estop = false; g_io.armed = true;
            g_io.last_setpoint_ms = millis();
            Serial.print("AK A: armed "); Serial.print((int)old);
            Serial.print(" -> ");         Serial.println((int)g_io.armed);
            break;
          }
          case 'I': case 'i': {
            bool old = g_io.armed;
            g_io.armed = false;
            Serial.print("AK I: armed "); Serial.print((int)old);
            Serial.print(" -> ");         Serial.println((int)g_io.armed);
            break;
          }
          case 'M': case 'm':
            g_io.req_characterise = true;
            Serial.println("AK M: characterise requested");
            break;
#if SENSOR_TYPE == SENSOR_TYPE_HALL
          case 'H': case 'h':
            g_req_hall_cal = true;
            Serial.println("AK H: hall-angle calibration requested (moteur désarmé)");
            break;
#endif
          case 'T': case 't': {
            float old = g_io.input_torque;
            g_io.control_mode = CTRL_TORQUE;   g_io.input_torque = v;
            g_io.last_setpoint_ms = millis();
            Serial.print("AK T: torque "); Serial.print(old, 2);
            Serial.print(" -> ");          Serial.print(v, 2);
            Serial.println(" Nm");
            break;
          }
          case 'V': case 'v': {
            float old = g_io.input_vel;
            g_io.control_mode = CTRL_VELOCITY; g_io.input_vel = v;
            g_io.last_setpoint_ms = millis();
            Serial.print("AK V: vel "); Serial.print(old, 2);
            Serial.print(" -> ");       Serial.print(v, 2);
            Serial.println(" rad/s");
            break;
          }
          case 'C': case 'c':
            g_io.req_clear_errors = true; g_io.estop = false;
            Serial.println("AK C: clear-errors requested");
            break;
          case 'K': case 'k': {
            // KP/KI/KD<val> : gains PID vitesse en Nm/(rad/s) ; 'K' seul
            // ré-applique et affiche les gains courants.
            v = atof(buf + 2);
            switch (buf[1]) {
              case 'P': case 'p': {
                float old = g_io.vel_gain;     g_io.vel_gain     = v;
                Serial.print("AK KP: vel_gain ");     Serial.print(old, 4);
                Serial.print(" -> ");                 Serial.println(v, 4);
                break;
              }
              case 'I': case 'i': {
                float old = g_io.vel_int_gain; g_io.vel_int_gain = v;
                Serial.print("AK KI: vel_int_gain "); Serial.print(old, 4);
                Serial.print(" -> ");                 Serial.println(v, 4);
                break;
              }
              case 'D': case 'd': {
                float old = g_io.vel_d_gain;   g_io.vel_d_gain   = v;
                Serial.print("AK KD: vel_d_gain ");   Serial.print(old, 5);
                Serial.print(" -> ");                 Serial.println(v, 5);
                break;
              }
              default:
                Serial.println("AK K: reapply vel gains");
                break;
            }
            g_io.req_vel_gains = true;
            break;
          }
          case 'J': case 'j': {
            // JP/JI/JD<val> : gains PID courant (V/A). 'J' seul ré-applique.
            v = atof(buf + 2);
            switch (buf[1]) {
              case 'P': case 'p': {
                float old = g_io.cur_p_gain;   g_io.cur_p_gain   = v;
                Serial.print("AK JP: cur_p ");   Serial.print(old, 4);
                Serial.print(" -> ");            Serial.println(v, 4);
                break;
              }
              case 'I': case 'i': {
                float old = g_io.cur_int_gain; g_io.cur_int_gain = v;
                Serial.print("AK JI: cur_i ");   Serial.print(old, 4);
                Serial.print(" -> ");            Serial.println(v, 4);
                break;
              }
              case 'D': case 'd': {
                float old = g_io.cur_d_gain;   g_io.cur_d_gain   = v;
                Serial.print("AK JD: cur_d ");   Serial.print(old, 5);
                Serial.print(" -> ");            Serial.println(v, 5);
                break;
              }
              default:
                Serial.println("AK J: reapply current gains");
                break;
            }
            g_io.req_cur_gains = true;
            break;
          }
          case 'P': case 'p': {
            // PP/PI/PD<val> : gains PID position. PP == G (P_angle.P).
            v = atof(buf + 2);
            switch (buf[1]) {
              case 'P': case 'p': {
                float old = g_io.pos_gain;     g_io.pos_gain     = (v > 0.0f) ? v : 0.0f;
                Serial.print("AK PP: pos_p ");   Serial.print(old, 4);
                Serial.print(" -> ");            Serial.println(g_io.pos_gain, 4);
                break;
              }
              case 'I': case 'i': {
                float old = g_io.pos_int_gain; g_io.pos_int_gain = v;
                Serial.print("AK PI: pos_i ");   Serial.print(old, 4);
                Serial.print(" -> ");            Serial.println(v, 4);
                break;
              }
              case 'D': case 'd': {
                float old = g_io.pos_d_gain;   g_io.pos_d_gain   = v;
                Serial.print("AK PD: pos_d ");   Serial.print(old, 5);
                Serial.print(" -> ");            Serial.println(v, 5);
                break;
              }
              default:
                Serial.println("AK P: reapply position gains");
                break;
            }
            g_io.req_pos_gains = true;
            break;
          }
          case 'L': case 'l': {
            // L<C|V><val> : limites runtime. Bornées à un plafond dur pour
            // qu'un client distant ne demande jamais une valeur dangereuse.
            v = atof(buf + 2);
            switch (buf[1]) {
              case 'C': case 'c': {
                float old = g_io.current_limit;
                g_io.current_limit = _constrain(v, 0.0f, CFG_CURRENT_LIMIT_MAX);
                Serial.print("AK LC: current_limit "); Serial.print(old, 2);
                Serial.print(" -> ");                   Serial.print(g_io.current_limit, 2);
                Serial.println(" A");
                break;
              }
              case 'V': case 'v': {
                float old = g_io.vel_limit;
                g_io.vel_limit = _constrain(v, 0.0f, CFG_VEL_LIMIT_MAX);
                Serial.print("AK LV: vel_limit "); Serial.print(old, 2);
                Serial.print(" -> ");               Serial.print(g_io.vel_limit, 2);
                Serial.println(" rad/s");
                break;
              }
              default:
                Serial.println("AK L?: use LC<A> or LV<rad/s>");
                break;
            }
            break;
          }
          case 'G': case 'g': {
            // G<val> : gain P de position (P_angle.P). Appliqué en boucle
            // fermée par applyControl si > 0.
            float old = g_io.pos_gain;
            g_io.pos_gain = (v > 0.0f) ? v : 0.0f;
            Serial.print("AK G: pos_gain "); Serial.print(old, 4);
            Serial.print(" -> ");            Serial.println(g_io.pos_gain, 4);
            break;
          }
          case 'X': case 'x': {
            // X<val> : consigne de position (rad) -> mode position.
            float old = g_io.input_pos;
            g_io.control_mode = CTRL_POSITION; g_io.input_pos = v;
            g_io.last_setpoint_ms = millis();
            Serial.print("AK X: pos "); Serial.print(old, 3);
            Serial.print(" -> ");       Serial.print(v, 3);
            Serial.println(" rad");
            break;
          }
          case 'Q': case 'q':
            // Q : dump de la config courante (ligne "cfg ...") pour le GUI.
            reportConfig();
            break;
          default:
          
            Serial.print("AK ?: unknown '"); Serial.print(buf[0]);
            Serial.println("'");
            break;
        }
      }
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

// ============================================================================
//  FreeRTOS fault hooks (configCHECK_FOR_STACK_OVERFLOW / configUSE_MALLOC_FAILED_HOOK
//  in include/STM32FreeRTOSConfig.h). Both conditions previously produced a
//  silent hang with no serial output at all -- these make the failure
//  self-report instead, and cut the gate driver since RTOS state can no
//  longer be trusted to keep running FOCTask/SafetyTask correctly.
// ============================================================================
extern "C" void vApplicationStackOverflowHandler(TaskHandle_t /*xTask*/, char *pcTaskName) {
  brake::off();
  digitalWrite(PIN_EN_GATE, LOW);
  Serial.print("\n[FATAL] Stack overflow in task \"");
  Serial.print(pcTaskName);
  Serial.println("\" -- halting. Increase its STACK_* in board_config.h.");
  Serial.flush();
  for (;;) {}
}

extern "C" void vApplicationMallocFailedHook(void) {
  brake::off();
  digitalWrite(PIN_EN_GATE, LOW);
  Serial.println("\n[FATAL] FreeRTOS heap allocation failed (configTOTAL_HEAP_SIZE exhausted) -- halting.");
  Serial.flush();
  for (;;) {}
}

static void SerialTask(void *) {
  uint32_t beat = 0;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    handleSerial();
    Serial.print("t=");         Serial.print(millis());
    Serial.print(" #");         Serial.print(beat++);
    Serial.print(" mode=");      Serial.print(g_io.control_mode);
    // Telemetry channels, generated from the schema shared with the web GUI
    // (include/telemetry_schema.h). Add a channel THERE and it streams here and
    // appears in the GUI's graphs automatically. The status/CAN fields below are
    // not plotted channels, so they stay hand-written.
    // obsdV/blnd touch the sensorless observer -> hall builds only.
#define TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr) \
    Serial.print(" " #key "="); Serial.print((float)(expr), prec);
#if SENSOR_TYPE == SENSOR_TYPE_HALL
#define TELEMETRY_CHANNEL_HALL TELEMETRY_CHANNEL
#else
#define TELEMETRY_CHANNEL_HALL(key, label, color, altkey, prec, expr)
#endif
#include "telemetry_schema.h"
#undef TELEMETRY_CHANNEL_HALL
#undef TELEMETRY_CHANNEL
    Serial.print(g_focReady ? " RUN" : (g_calibrated ? " idle" : " SAFE"));
    Serial.print(g_fault ? " [FAULT]" : "");
    if (g_io.axis_error) { Serial.print(" err=0x"); Serial.print(g_io.axis_error, HEX); }
    Serial.print(" can_tx_ok=");   Serial.print(g_can.txOkCount());
    Serial.print(" can_tx_fail="); Serial.print(g_can.txFailCount());
    Serial.print(" can_rx=");      Serial.println(g_can.rxCount());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

// ============================================================================
//  Setup Initialization
// ============================================================================
void setup() {
  // Demi-pont AUX (frein) : gates BAS immédiatement, AVANT toute bascule en
  // fonction alternative, pour que le demi-pont ne passe jamais par un état
  // indéterminé au démarrage. brake::init() prend la main plus bas.
  pinMode(PIN_AUX_H, OUTPUT); digitalWrite(PIN_AUX_H, LOW);
  pinMode(PIN_AUX_L, OUTPUT); digitalWrite(PIN_AUX_L, LOW);

  pinMode(PIN_M1_CS, OUTPUT); digitalWrite(PIN_M1_CS, HIGH); // Disable unused M1 SPI
  pinMode(PIN_N_FAULT, INPUT_PULLUP);
  analogReadResolution(12);

  // DRV8301 Hardware Reset
  pinMode(PIN_EN_GATE, OUTPUT);
  digitalWrite(PIN_EN_GATE, LOW);  delay(50);
  digitalWrite(PIN_EN_GATE, HIGH); delay(50);
  
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- SimpleFOC + FreeRTOS + CANSimple ---");

  // DRV8301 Initialization
  drv.begin();
  bool gain_ok = drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
  Serial.print("DRV8301 status1=0x"); Serial.print(drv.status1(), HEX);
  Serial.print(" gain_set="); Serial.println(gain_ok ? "OK" : "FAIL(check SPI)");

  // Sensor Initialization
  sensor.init();
#if SENSOR_TYPE == SENSOR_TYPE_HALL
  sensor.enableInterrupts(doHallA, doHallB, doHallC);
  // Force multi-edge velocity averaging (see HallSensorSmoothVel.h) instead
  // of Sensor's library default 100us (effectively single-edge at our poll rate).
  sensor.min_elapsed_time = CFG_HALL_VEL_WINDOW;
  // Corrections d'angle de secteur hall pré-calibrées (voir board_config.h +
  // commande série 'H'). Sinon offsets nuls/inactifs jusqu'à un 'H' en session.
  if (CFG_HALL_PRECALIBRATED) {
    const float hall_cal[6] = CFG_HALL_CAL_OFFSETS;
    for (int i = 0; i < 6; i++) sensor.sector_offset[i] = hall_cal[i];
    sensor.offsets_active = true;
    Serial.println("Hall sector offsets: PRECALIBRATED (CFG_HALL_CAL_OFFSETS actifs)");
  }
#endif
  motor.linkSensor(&foc_sensor);

  // Driver Configuration
  driver.voltage_power_supply = CFG_VBUS_NOMINAL;
  driver.pwm_frequency        = CFG_PWM_FREQ_HZ;
  driver.voltage_limit        = CFG_VOLT_LIMIT;
  if (!driver.init()) { Serial.println("[-] driver.init FAILED"); while (1); }
  motor.linkDriver(&driver);

  // Lowside Current Sensing Configuration
  current_sense.linkDriver(&driver);
  // skip_align=true SAUTE la vérification polarité/pins des shunts par initFOC.
  // On ne la saute qu'une fois la config validée et figée (CFG_PRECALIBRATED) :
  // une polarité inversée en foc_current = contre-réaction positive (emballement).
  current_sense.skip_align = (CFG_PRECALIBRATED != 0);
  g_iSenseOk = (current_sense.init() == 1);
  if (g_iSenseOk) {
    motor.linkCurrentSense(&current_sense);
    // Boucle de courant fermée : le PID vitesse sort des ampères et
    // motor.current_limit devient effectif (en voltage il était ignoré).
    motor.torque_controller = TorqueControlType::foc_current;
    motor.PID_current_q.P = CFG_CUR_P; motor.PID_current_q.I = CFG_CUR_I;
    motor.PID_current_d.P = CFG_CUR_P; motor.PID_current_d.I = CFG_CUR_I;
    motor.LPF_current_q.Tf = CFG_LPF_CUR_TF;
    motor.LPF_current_d.Tf = CFG_LPF_CUR_TF;
    Serial.println("Current sense OK -> foc_current torque control");
  } else {
    motor.torque_controller = TorqueControlType::voltage;
    Serial.println("[!] current_sense.init FAILED -> voltage-torque fallback");
  }

  // Motor & Parameters Configuration
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
  // Observateur sensorless : flux_linkage (Wb) dérivé de KV/pp. Le constructeur
  // de l'observateur tourne avant setup() (KV_rating pas encore posé) -> ici.
  hybrid._obs.flux_linkage = 60.0f / (_SQRT3 * _PI * CFG_KV * (float)CFG_POLE_PAIRS * 2.0f);
  hybrid._obs.min_elapsed_time = CFG_HALL_VEL_WINDOW;  // même fenêtre que le hall
                                          // -> obsdV = vrai écart de suivi, pas un
                                          //    artefact de fenêtre, et v_obs lissé
  hybrid.vel_lo  = CFG_SENSORLESS_VEL_LO;
  hybrid.vel_hi  = CFG_SENSORLESS_VEL_HI;
  hybrid.enabled = (CFG_SENSORLESS_ENABLE != 0) && g_iSenseOk;  // besoin du current-sense
  hybrid.initObserver();
#endif

  if (!motor.init()) { Serial.println("[-] motor.init FAILED"); while (1); }
  motor.disable(); // Force initial safe state

  // Global variables initialization
  g_io.armed         = false;
  g_io.estop         = false;
  g_io.control_mode  = CTRL_TORQUE;
  g_io.input_torque  = 0.0f;
  g_io.input_vel     = 0.0f;
  g_io.vel_limit     = CFG_VEL_LIMIT;
  g_io.current_limit = CFG_CURRENT_LIMIT;
  g_io.pos_gain      = CFG_POS_P;   // miroir du gain P position (commande G/PP)
  g_io.pos_int_gain  = CFG_POS_I;
  g_io.pos_d_gain    = CFG_POS_D;
  g_io.cur_p_gain    = CFG_CUR_P;   // miroirs des gains courant (V/A, sans Kt)
  g_io.cur_int_gain  = CFG_CUR_I;
  g_io.cur_d_gain    = CFG_CUR_D;
  // Miroirs des gains vitesse en Nm/(rad/s), inverse exact de l'application
  // dans applyControl() (les CFG_* sont en A/(rad/s), ou en V en fallback)
  {
    float k = g_iSenseOk ? CFG_KT : 1.0f;
    g_io.vel_gain     = CFG_VEL_P * k;
    g_io.vel_int_gain = CFG_VEL_I * k;
    g_io.vel_d_gain   = CFG_VEL_D * k;
  }
  g_io.last_setpoint_ms = millis();

  // Chopper de dissipation (sort ARRÊTÉ de init()) + config ADC Vbus
  // (480 cycles) AVANT le lancement de SafetyTask qui lit à 1 kHz.
  // brake::init() est appelé après le reset du DRV8301 ci-dessus : GVDD, qui
  // alimente le LM5109B, n'existe que DRV8301 réveillé.
  brake::init();
  vbusAdcInit();
  Serial.print("Brake resistor: "); Serial.print(CFG_BRAKE_R, 1);
#if CFG_BUS_SOURCE == CFG_BUS_SOURCE_PSU
  Serial.print(" ohm on AUX [PSU], chopper ");
#else
  Serial.print(" ohm on AUX [BATTERY], chopper ");
#endif
  Serial.print(CFG_BRAKE_VBUS_OFF, 1);
  Serial.print("/");                  Serial.print(CFG_BRAKE_VBUS_ON, 1);
  Serial.print(" V gain ");           Serial.print(CFG_BRAKE_GAIN, 2);
  Serial.print(", regen derate ");    Serial.print(CFG_VBUS_REGEN_START, 1);
  Serial.print("-");                  Serial.print(CFG_VBUS_REGEN_FULL, 1);
  Serial.print(" V, OV trip ");       Serial.print(CFG_VBUS_OV_TRIP, 1);
  Serial.println(" V");

  // CAN Simple Bus Start
  g_can.begin(CFG_CAN_NODE_ID, CFG_CAN_BAUD, NVIC_PRIO_RTOS_SAFE);
  Serial.print("CAN up: node "); Serial.print(CFG_CAN_NODE_ID);
  Serial.print(" @ "); Serial.print(CFG_CAN_BAUD); Serial.println(" bps");

  // Task Creation
  BaseType_t r1 = xTaskCreate(SafetyTask, "SAFE",  STACK_SAFETY,    NULL, PRIO_SAFETY,    NULL);
  BaseType_t r2 = xTaskCreate(FOCTask,    "FOC",   STACK_FOC,       NULL, PRIO_FOC,       NULL);
  BaseType_t r3 = xTaskCreate(CommsTask,  "COMMS", STACK_COMMS,     NULL, PRIO_COMMS,     NULL);
  BaseType_t r4 = xTaskCreate(SerialTask, "SER",   STACK_TELEMETRY, NULL, PRIO_TELEMETRY, NULL);
  if (r1 != pdPASS || r2 != pdPASS || r3 != pdPASS || r4 != pdPASS) {
    Serial.println("[-] xTaskCreate FAILED (Check FreeRTOS heap)");
    while (1);
  }

  Serial.println("SAFE state (disarmed). Send 'A' via serial or CAN CLOSED_LOOP state to arm.");
#if SENSOR_TYPE == SENSOR_TYPE_HALL
  Serial.println("Serial cmds: A arm | I idle | V<rad/s> | T<Nm> | X<rad> pos | M charac R/L | H hall-cal | C clear | KP/KI/KD<v> vel PID | K show");
#else
  Serial.println("Serial cmds: A arm | I idle | V<rad/s> | T<Nm> | X<rad> pos | M charac R/L | C clear | KP/KI/KD<v> vel PID | K show");
#endif
  Serial.println("  gains:     KP/KI/KD<v> vel | JP/JI/JD<v> current | PP/PI/PD<v> position");
  Serial.println("  config:    LC<A> current-lim | LV<rad/s> vel-lim | G<v> pos-gain | Q dump config (cfg ...)");
  vTaskStartScheduler();
  for (;;) {}
}

void loop() {}