// ============================================================================
//  SimpleFOC + FreeRTOS + ODrive CANSimple  —  ODrive v3.6 (MKS clone) / F405
// ============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <STM32FreeRTOS.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h" 
#include "drv8301.h"
#include "odrive_can.h"
#include "board_config.h"

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
}

// ============================================================================
//  Objects & Hardware Association
// ============================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_M0_INH_A, PIN_M0_INL_A,
                                       PIN_M0_INH_B, PIN_M0_INL_B,
                                       PIN_M0_INH_C, PIN_M0_INL_C, PIN_EN_GATE);
BLDCMotor motor = BLDCMotor(CFG_POLE_PAIRS);

#if SENSOR_TYPE == SENSOR_TYPE_HALL
HallSensor sensor = HallSensor(PIN_ENC_A, PIN_ENC_B, PIN_ENC_Z, CFG_POLE_PAIRS);
static void doHallA() { sensor.handleA(); }
static void doHallB() { sensor.handleB(); }
static void doHallC() { sensor.handleC(); }
#else
STM32HWEncoder sensor = STM32HWEncoder(CFG_ENC_PPR, PIN_ENC_A, PIN_ENC_B);
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
static   bool  g_iSenseOk      = false;
static   TaskHandle_t g_focTask = nullptr;
static   HardwareTimer *g_focTimer = nullptr;

// 20 kHz FOC tick ISR
static void onFocTick() {
  BaseType_t hpw = pdFALSE;
  if (g_focTask) vTaskNotifyGiveFromISR(g_focTask, &hpw);
  portYIELD_FROM_ISR(hpw);
}

// ============================================================================
//  Logic Helpers
// ============================================================================
static float readVbus() {
  return (float)analogRead(PIN_VBUS) * (3.3f / 4095.0f) * CFG_VBUS_DIV;
}

static void enableStage() {
  motor.enable(); 
  delay(5); // Setup time for DRV8301 SPI stability
  drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
}

// Axis State Machine (1 kHz)
static void applyControl() {
  uint32_t now = millis();

  if (g_io.req_reboot) {
    motor.disable(); digitalWrite(PIN_EN_GATE, LOW);
    NVIC_SystemReset();
  }
  
  if (g_io.req_clear_errors) {
    g_io.req_clear_errors = false;
    g_fault = false;
    g_io.axis_error = 0;
    Serial.println("[OK] Erreurs réinitialisées.");
  }
  
  // Sécurité globale simplifiée (la SafetyTask gère le hardware)
  bool safe = !g_io.estop && !g_fault;

  // --- MOTOR_CALIBRATION (R/L Measurement) ---
  if (g_io.req_characterise && !g_focReady) {
    g_io.req_characterise = false;
    if (g_iSenseOk && safe) {
      Serial.println("Characterising motor (R/L)...");
      enableStage();
      motor.characteriseMotor(CFG_CHAR_VOLTAGE);
      motor.disable();
      Serial.print("  R = "); Serial.print(motor.phase_resistance, 4);
      Serial.print(" ohm   L = "); Serial.print(motor.phase_inductance * 1e6f, 2);
      Serial.println(" uH");
    } else {
      Serial.println("[!] Characterise requires current sensing + safe state");
    }
    return;
  }

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
      case CTRL_VELOCITY:
        motor.controller = MotionControlType::velocity;
        g_active_target  = g_io.input_vel;
        break;
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
    if (g_io.vel_limit     > 0.0f) motor.velocity_limit = g_io.vel_limit;
    if (g_io.current_limit > 0.0f) motor.current_limit  = g_io.current_limit;
    if (g_io.pos_gain      > 0.0f) motor.P_angle.P      = g_io.pos_gain;

    if (CFG_WATCHDOG_MS > 0 && (now - g_io.last_setpoint_ms) > CFG_WATCHDOG_MS) {
      g_io.axis_error |= ERR_WATCHDOG_EXPIRED;
      g_io.armed = false;
    }
  } else {
    g_active_target = 0.0f;
  }
}

static void publishTelemetry() {
  g_io.pos_rev = motor.shaft_angle / TWO_PI;
  g_io.vel_rev = motor.shaft_velocity / TWO_PI;
  g_io.vbus    = readVbus();
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
    
    // Forcer la lecture du capteur pour rafraîchir 'vel' même au repos (SAFE/idle)
    sensor.update();
    motor.shaft_velocity = sensor.getVelocity();
    motor.shaft_angle = sensor.getMechanicalAngle();

    if (!g_focReady || g_fault) continue;
    motor.loopFOC();
    motor.move(g_active_target);
  }
}

// SafetyTask (1 kHz - Hardware fault handler)
static void SafetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t fault_counter = 0;

  for (;;) {
    // Vérification de nFAULT uniquement si le driver est censé être actif
    if (digitalRead(PIN_EN_GATE) == HIGH && digitalRead(PIN_N_FAULT) == LOW) {
      fault_counter++;
      if (fault_counter > 10) { // Anti-rebond : 10 ms consécutives à bas
        digitalWrite(PIN_EN_GATE, LOW); // Coupure d'urgence immédiate
        g_fault = true;
        g_io.axis_error |= ERR_MOTOR_FAILED;
      }
    } else {
      fault_counter = 0; // Reset si le signal revient au vert ou driver éteint
    }
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
        switch (buf[0]) {
          case 'A': case 'a':
            g_io.estop = false; g_io.armed = true;
            g_io.last_setpoint_ms = millis(); break;
          case 'I': case 'i':
            g_io.armed = false; break;
          case 'M': case 'm':
            g_io.req_characterise = true; break;
          case 'T': case 't':
            g_io.control_mode = CTRL_TORQUE;   g_io.input_torque = v;
            g_io.last_setpoint_ms = millis(); break;
          case 'V': case 'v':
            g_io.control_mode = CTRL_VELOCITY; g_io.input_vel = v;
            g_io.last_setpoint_ms = millis(); break;
          case 'C': case 'c':
            g_io.req_clear_errors = true; g_io.estop = false; break;
        }
      }
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

static void SerialTask(void *) {
  uint32_t beat = 0;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    handleSerial();
    Serial.print("t=");         Serial.print(millis());
    Serial.print(" #");         Serial.print(beat++);
    Serial.print(" mode=");      Serial.print(g_io.control_mode);
    Serial.print(" tgt=");       Serial.print(g_active_target, 2);
    Serial.print(" Iq=");        Serial.print(g_io.iq_measured, 2);
    Serial.print(" vel=");       Serial.print(motor.shaft_velocity, 2);
    Serial.print(" Vbus=");      Serial.print(g_io.vbus, 1);
    Serial.print(g_focReady ? " RUN" : (g_calibrated ? " idle" : " SAFE"));
    Serial.println(g_fault ? " [FAULT]" : "");
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

// ============================================================================
//  Setup Initialization
// ============================================================================
void setup() {
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
#endif
  motor.linkSensor(&sensor);

  // Driver Configuration
  driver.voltage_power_supply = CFG_VBUS_NOMINAL;
  driver.pwm_frequency        = CFG_PWM_FREQ_HZ;
  driver.voltage_limit        = CFG_VOLT_LIMIT;
  if (!driver.init()) { Serial.println("[-] driver.init FAILED"); while (1); }
  motor.linkDriver(&driver);

  // Lowside Current Sensing Configuration
  current_sense.linkDriver(&driver);
  current_sense.skip_align = false; // On force l'alignement pour la mesure R/L
  g_iSenseOk = (current_sense.init() == 1);
  if (g_iSenseOk) {
    motor.linkCurrentSense(&current_sense);
    motor.torque_controller = TorqueControlType::voltage; // On commence par le voltage pour la mesure R/L
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
  motor.controller           = MotionControlType::velocity_openloop;
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
  g_io.last_setpoint_ms = millis();

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
  vTaskStartScheduler();
  for (;;) {}
}

void loop() {}