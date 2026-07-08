// ============================================================================
//  SimpleFOC + FreeRTOS + ODrive CANSimple  —  ODrive v3.6 (MKS clone) / F405
//
//  SAFE-STATE BOOT: powers up DISARMED — driver off, motor free, zero torque.
//  Does NOTHING until armed by CAN Set_Axis_State(CLOSED_LOOP) (or serial 'A').
//  First arm runs sensor calibration (initFOC); disarm / E-stop / DRV fault drop
//  back to the safe state.
//
//  Phase 4 — CURRENT SENSING: the DRV8301 shunt amplifiers are read via
//  LowsideCurrentSense, giving true FOC current control (torque in Amps/Nm) with
//  a current limit. Torque over CAN is Nm, converted to Iq via Kt = 8.27/KV.
//  MOTOR_CALIBRATION (CAN state 4 / serial 'M') measures phase R and L.
//  If current-sense init fails, the firmware falls back to voltage-mode torque.
//
//  Tasks: FOCTask(4, 20 kHz) · SafetyTask(5, nFAULT) · CommsTask(3, CAN+FSM) ·
//         SerialTask(2, console).
//  Console (115200): A=arm  I=idle  V<rad/s>  T<Nm>  M=measure R/L  C=clear
// ============================================================================
#include <Arduino.h>
#include <SimpleFOC.h>
#include <STM32FreeRTOS.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"   // SimpleFOCDrivers
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
//  Objects
// ============================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_M0_INH_A, PIN_M0_INL_A,
                                       PIN_M0_INH_B, PIN_M0_INL_B,
                                       PIN_M0_INH_C, PIN_M0_INL_C, PIN_EN_GATE);
BLDCMotor      motor  = BLDCMotor(CFG_POLE_PAIRS);

// Feedback sensor — selected at compile time in board_config.h.
#if SENSOR_TYPE == SENSOR_TYPE_HALL
HallSensor     sensor = HallSensor(PIN_ENC_A, PIN_ENC_B, PIN_ENC_Z, CFG_POLE_PAIRS);
static void doHallA() { sensor.handleA(); }
static void doHallB() { sensor.handleB(); }
static void doHallC() { sensor.handleC(); }
#else
STM32HWEncoder sensor = STM32HWEncoder(CFG_ENC_PPR, PIN_ENC_A, PIN_ENC_B);
#endif

// DRV8301 config SPI (SPI3) + low-side current sense (phases B/C on PC0/PC1).
SPIClass       spi3(PIN_DRV_MOSI, PIN_DRV_MISO, PIN_DRV_SCK);
DRV8301        drv(spi3, PIN_M0_CS);
LowsideCurrentSense current_sense =
    LowsideCurrentSense(CFG_SHUNT_OHMS, CFG_DRV_GAIN, _NC, PIN_M0_IB, PIN_M0_IC);

AxisIO         g_io;            // shared command/telemetry block
OdriveCAN      g_can(g_io);     // CANSimple interface (CAN1 PB8/PB9)

// ============================================================================
//  Shared state
// ============================================================================
volatile float        g_active_target = 0.0f;   // consumed by move() in FOCTask
volatile bool         g_fault         = false;   // DRV8301 hardware fault latch
volatile bool         g_focReady      = false;   // FOCTask may drive (armed+calibrated)
static   bool         g_calibrated    = false;   // initFOC has succeeded once
static   bool         g_iSenseOk      = false;   // current sensing active (else voltage)
static   TaskHandle_t g_focTask       = nullptr;
static   HardwareTimer *g_focTimer    = nullptr;

// ============================================================================
//  20 kHz FOC tick ISR -> wake FOCTask  (syscall-safe NVIC priority)
// ============================================================================
static void onFocTick() {
  BaseType_t hpw = pdFALSE;
  if (g_focTask) vTaskNotifyGiveFromISR(g_focTask, &hpw);
  portYIELD_FROM_ISR(hpw);
}

// ============================================================================
//  Helpers
// ============================================================================
static float readVbus() {
  // 12-bit ADC, 3.3 V ref, external divider CFG_VBUS_DIV. VERIFY divider!
  return (float)analogRead(PIN_VBUS) * (3.3f / 4095.0f) * CFG_VBUS_DIV;
}

// Enable the stage. Re-program the DRV8301 gain each time because EN_GATE was
// pulled low in the safe state (the DRV may reset its registers on wake).
static void enableStage() {
  motor.enable();                 // driver.enable() -> EN_GATE high
  delay(2);                       // let the DRV wake before SPI
  drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
}

// Axis state machine + command bridge. Runs at 1 kHz in CommsTask.
static void applyControl() {
  uint32_t now = millis();

  if (g_io.req_reboot) {
    motor.disable(); digitalWrite(PIN_EN_GATE, LOW);
    NVIC_SystemReset();
  }
  if (g_io.req_clear_errors) {
    g_io.req_clear_errors = false;
    if (digitalRead(PIN_N_FAULT) == HIGH) g_fault = false;
    g_io.axis_error = 0;
  }

  bool safe = !g_io.estop && !g_fault && (digitalRead(PIN_N_FAULT) == HIGH);

  // --- MOTOR_CALIBRATION: measure phase R/L (only while disarmed & safe) ---
  if (g_io.req_characterise && !g_focReady) {
    g_io.req_characterise = false;
    if (g_iSenseOk && safe) {
      Serial.println("Characterising motor (R/L)...");
      enableStage();
      motor.characteriseMotor(CFG_CHAR_VOLTAGE);   // sets phase_resistance/inductance
      motor.disable();
      Serial.print("  R = "); Serial.print(motor.phase_resistance, 4);
      Serial.print(" ohm   L = "); Serial.print(motor.phase_inductance * 1e6f, 2);
      Serial.println(" uH");
    } else {
      Serial.println("[!] characterise needs current sensing + safe state");
    }
    return;
  }

  bool want = g_io.armed && safe;

  // --- DISARM: return to the safe state ---
  if (!want && g_focReady) {
    g_focReady = false;
    g_active_target = 0.0f;
    motor.disable();
  }

  // --- ARM: calibrate on first arm, then enable closed loop ---
  //  g_focReady is still false here, so FOCTask stays idle while initFOC()
  //  (which drives the motor itself) runs uninterrupted.
  if (want && !g_focReady) {
    enableStage();
    if (!g_calibrated) {
      // Pre-calibrated: pre-load the saved sensor offset/direction so initFOC
      // skips the (large) alignment SEARCH — SimpleFOC honours these if set.
      // Otherwise auto-align (the motor sweeps to find them).
      if (CFG_PRECALIBRATED) {
        motor.sensor_direction    = (CFG_SENSOR_DIRECTION >= 0) ? Direction::CW
                                                                : Direction::CCW;
        motor.zero_electric_angle = CFG_ZERO_ELEC_ANGLE;
      }
      int ok = motor.initFOC();
      if (ok) {
        g_calibrated = true;
        // Print the calibration result so it can be copied into board_config.h.
        Serial.print("initFOC OK | CFG_SENSOR_DIRECTION=");
        Serial.print(motor.sensor_direction == Direction::CW ? 1 : -1);
        Serial.print("  CFG_ZERO_ELEC_ANGLE=");
        Serial.println(motor.zero_electric_angle, 4);
      } else {
        g_io.axis_error |= ERR_ENCODER_FAILED;   // alignment failed
        g_io.armed = false;
        motor.disable();
        return;
      }
    }
    g_focReady = true;
  }

  // --- while running: apply mode / target / limits / watchdog ---
  if (g_focReady) {
    switch (g_io.control_mode) {
      case CTRL_VELOCITY:
        motor.controller = MotionControlType::velocity;
        g_active_target  = g_io.input_vel;                    // rad/s
        break;
      case CTRL_POSITION:
        motor.controller = MotionControlType::angle;
        g_active_target  = g_io.input_pos;                    // rad
        break;
      case CTRL_TORQUE:
      case CTRL_VOLTAGE:
      default:
        motor.controller = MotionControlType::torque;
        // foc_current: target is Iq in Amps (Nm/Kt). Voltage fallback: Nm as Uq.
        g_active_target = g_iSenseOk ? (g_io.input_torque / CFG_KT)
                                     :  g_io.input_torque;
        break;
    }
    if (g_io.vel_limit     > 0.0f) motor.velocity_limit = g_io.vel_limit;
    if (g_io.current_limit > 0.0f) motor.current_limit  = g_io.current_limit;
    if (g_io.pos_gain      > 0.0f) motor.P_angle.P      = g_io.pos_gain;

    if (CFG_WATCHDOG_MS > 0 &&
        (now - g_io.last_setpoint_ms) > CFG_WATCHDOG_MS) {
      g_io.axis_error |= ERR_WATCHDOG_EXPIRED;
      g_io.armed = false;                        // -> disarm next iteration
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
    g_io.iq_setpoint = motor.current_sp;                 // A
    g_io.iq_measured = motor.current.q;                  // A
    float p = motor.voltage.q * motor.current.q + motor.voltage.d * motor.current.d;
    g_io.ibus = (g_io.vbus > 1.0f) ? (p / g_io.vbus) : 0.0f;   // estimated bus current
  } else {
    g_io.iq_setpoint = 0.0f;
    g_io.iq_measured = 0.0f;
    g_io.ibus        = 0.0f;
  }
  g_io.cur_state = g_focReady ? AXIS_CLOSED_LOOP : AXIS_IDLE;
}

// ============================================================================
//  FOCTask
// ============================================================================
static void FOCTask(void *) {
  g_focTask = xTaskGetCurrentTaskHandle();
  g_focTimer = new HardwareTimer(TIM6);   // free basic timer (TIM1=PWM, TIM3=enc)
  g_focTimer->setOverflow(FOC_TICK_HZ, HERTZ_FORMAT);
  g_focTimer->attachInterrupt(onFocTick);
  g_focTimer->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);
  g_focTimer->resume();

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);     // block until next 20 kHz tick
    if (!g_focReady || g_fault) continue;        // disarmed/faulted => idle
    motor.loopFOC();
    motor.move(g_active_target);
  }
}

// ============================================================================
//  SafetyTask
// ============================================================================
static void SafetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (digitalRead(PIN_N_FAULT) == LOW) {
      digitalWrite(PIN_EN_GATE, LOW);            // definitive hardware cut
      g_fault = true;
      g_io.axis_error |= ERR_MOTOR_FAILED;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

// ============================================================================
//  CommsTask — CAN drain + state machine + telemetry (1 kHz)
// ============================================================================
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

// ============================================================================
//  SerialTask — debug + command console
// ============================================================================
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
          case 'A': case 'a':                    // arm (like Set_Axis_State(8))
            g_io.estop = false; g_io.armed = true;
            g_io.last_setpoint_ms = millis(); break;
          case 'I': case 'i':                    // idle / disarm
            g_io.armed = false; break;
          case 'M': case 'm':                    // measure phase R/L
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
    Serial.print("t=");     Serial.print(millis());
    Serial.print(" #");     Serial.print(beat++);
    Serial.print(" mode=");  Serial.print(g_io.control_mode);
    Serial.print(" tgt=");   Serial.print(g_active_target, 2);
    Serial.print(" Iq=");    Serial.print(g_io.iq_measured, 2);
    Serial.print(" vel=");   Serial.print(motor.shaft_velocity, 2);
    Serial.print(" Vbus=");  Serial.print(g_io.vbus, 1);
    Serial.print(g_focReady ? " RUN" : (g_calibrated ? " idle" : " SAFE"));
    Serial.println(g_fault ? " [FAULT]" : "");
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

// ============================================================================
//  setup — configure only; DO NOT enable/calibrate/spin. Wait for arm.
// ============================================================================
void setup() {
  pinMode(PIN_M1_CS, OUTPUT); digitalWrite(PIN_M1_CS, HIGH);   // M1 DRV unused
  pinMode(PIN_N_FAULT, INPUT_PULLUP);
  analogReadResolution(12);

  // DRV8301 hardware reset; leave EN_GATE HIGH for SPI config + offset cal.
  pinMode(PIN_EN_GATE, OUTPUT);
  digitalWrite(PIN_EN_GATE, LOW);  delay(50);
  digitalWrite(PIN_EN_GATE, HIGH); delay(50);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- SimpleFOC + FreeRTOS + CANSimple (current sensing) ---");

  // ---- DRV8301: SPI + shunt-amp gain ----
  drv.begin();
  bool gain_ok = drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
  Serial.print("DRV8301 status1=0x"); Serial.print(drv.status1(), HEX);
  Serial.print(" gain_set="); Serial.println(gain_ok ? "OK" : "FAIL(check SPI)");

  // ---- sensor: quadrature (HW timer, no EXTI) or hall (interrupts) ----
  sensor.init();
#if SENSOR_TYPE == SENSOR_TYPE_HALL
  sensor.enableInterrupts(doHallA, doHallB, doHallC);   // low-rate: scheduler-safe
#endif
  motor.linkSensor(&sensor);

  // ---- driver (configured, NOT enabled) ----
  driver.voltage_power_supply = CFG_VBUS_NOMINAL;
  driver.pwm_frequency        = CFG_PWM_FREQ_HZ;
  driver.voltage_limit        = CFG_VOLT_LIMIT;
  if (!driver.init()) { Serial.println("[-] driver.init FAILED"); while (1); }
  motor.linkDriver(&driver);

  // ---- low-side current sense (offset calibration needs zero current) ----
  current_sense.linkDriver(&driver);
  g_iSenseOk = (current_sense.init() == 1);
  if (g_iSenseOk) {
    motor.linkCurrentSense(&current_sense);
    motor.torque_controller = TorqueControlType::foc_current;   // true Amps/Nm
    motor.PID_current_q.P = CFG_CUR_P; motor.PID_current_q.I = CFG_CUR_I;
    motor.PID_current_d.P = CFG_CUR_P; motor.PID_current_d.I = CFG_CUR_I;
    motor.LPF_current_q.Tf = CFG_LPF_CUR_TF;
    motor.LPF_current_d.Tf = CFG_LPF_CUR_TF;
    Serial.println("Current sense OK -> foc_current torque control");
  } else {
    motor.torque_controller = TorqueControlType::voltage;        // fallback
    Serial.println("[!] current_sense.init FAILED -> voltage-torque fallback");
  }

  // ---- motor / control config (no calibration here) ----
  motor.voltage_limit        = CFG_VOLT_LIMIT;
  motor.current_limit        = CFG_CURRENT_LIMIT;
  motor.velocity_limit       = CFG_VEL_LIMIT;
  motor.controller           = MotionControlType::torque;
  motor.foc_modulation       = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = CFG_VOLT_ALIGN;
  motor.motion_downsample    = MOTION_DOWNSAMPLE;
  motor.PID_velocity.P       = CFG_VEL_P;
  motor.PID_velocity.I       = CFG_VEL_I;
  motor.PID_velocity.D       = CFG_VEL_D;
  motor.PID_velocity.output_ramp = CFG_VEL_RAMP;
  motor.P_angle.P            = CFG_POS_P;
  motor.LPF_velocity.Tf      = CFG_LPF_VEL_TF;
  // Saved phase R/L from characteriseMotor (0 = leave unset -> SimpleFOC defaults).
  if (CFG_PHASE_R > 0.0f) motor.phase_resistance = CFG_PHASE_R;
  if (CFG_PHASE_L > 0.0f) motor.phase_inductance = CFG_PHASE_L;
  if (!motor.init()) { Serial.println("[-] motor.init FAILED"); while (1); }
  motor.disable();                                            // EN_GATE low -> safe

  // ---- default command state: SAFE (disarmed, zero setpoint) ----
  g_io.armed         = false;
  g_io.estop         = false;
  g_io.control_mode  = CTRL_TORQUE;
  g_io.input_torque  = 0.0f;
  g_io.input_vel     = 0.0f;
  g_io.vel_limit     = CFG_VEL_LIMIT;
  g_io.current_limit = CFG_CURRENT_LIMIT;
  g_io.last_setpoint_ms = millis();

  // ---- CAN ----
  g_can.begin(CFG_CAN_NODE_ID, CFG_CAN_BAUD, NVIC_PRIO_RTOS_SAFE);
  Serial.print("CAN up: node "); Serial.print(CFG_CAN_NODE_ID);
  Serial.print(" @ "); Serial.print(CFG_CAN_BAUD); Serial.println(" bps");

  // ---- launch FreeRTOS ----
  BaseType_t r1 = xTaskCreate(SafetyTask, "SAFE",  STACK_SAFETY,    NULL, PRIO_SAFETY,    NULL);
  BaseType_t r2 = xTaskCreate(FOCTask,    "FOC",   STACK_FOC,       NULL, PRIO_FOC,       NULL);
  BaseType_t r3 = xTaskCreate(CommsTask,  "COMMS", STACK_COMMS,     NULL, PRIO_COMMS,     NULL);
  BaseType_t r4 = xTaskCreate(SerialTask, "SER",   STACK_TELEMETRY, NULL, PRIO_TELEMETRY, NULL);
  if (r1 != pdPASS || r2 != pdPASS || r3 != pdPASS || r4 != pdPASS) {
    Serial.println("[-] xTaskCreate FAILED (bump FreeRTOS heap)");
    while (1);
  }

  Serial.println("SAFE state (disarmed). Arm to calibrate + run:");
  Serial.println("  CAN: Set_Axis_State(8)   or   serial: A  (M = measure R/L)");
  vTaskStartScheduler();
  for (;;) {}
}

void loop() {}
