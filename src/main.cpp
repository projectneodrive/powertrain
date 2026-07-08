// ============================================================================
//  SimpleFOC + FreeRTOS + ODrive CANSimple  —  ODrive v3.6 (MKS clone) / F405
//
//  SAFE-STATE BOOT: the board powers up DISARMED — driver off, motor free, zero
//  torque, no calibration motion. It does NOTHING until it is armed by a CAN
//  Set_Axis_State(CLOSED_LOOP) (or the serial 'A' command). On the first arm it
//  runs sensor calibration (initFOC), then enters closed loop. Disarming, an
//  E-stop, or a DRV8301 fault immediately drops back to the safe state.
//
//  Tasks:
//   * FOCTask   (prio 4): woken by a 20 kHz TIM6 tick; runs loopFOC()+move()
//                ONLY while g_focReady (armed & calibrated). STM32HWEncoder
//                (TIM3) => no EXTI => no scheduler starvation.
//   * SafetyTask(prio 5): polls DRV8301 nFAULT, hardware-cuts EN_GATE.
//   * CommsTask (prio 3): drains CAN @1 kHz, runs the axis state machine
//                (arm/calibrate/disarm) + watchdog, publishes telemetry, sends
//                cyclic CAN frames.
//   * SerialTask(prio 2): USB-CDC debug + a tiny command console.
//
//  Serial console (115200): A=arm  I=idle/disarm  V<rad/s>=velocity
//                           T<Nm/V>=torque  C=clear errors
// ============================================================================
#include <Arduino.h>
#include <SimpleFOC.h>
#include <STM32FreeRTOS.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"   // SimpleFOCDrivers
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
STM32HWEncoder encoder = STM32HWEncoder(CFG_ENC_PPR, PIN_ENC_A, PIN_ENC_B);

AxisIO         g_io;            // shared command/telemetry block
OdriveCAN      g_can(g_io);     // CANSimple interface (CAN1 PB8/PB9)

// ============================================================================
//  Shared state
// ============================================================================
volatile float        g_active_target = 0.0f;   // consumed by move() in FOCTask
volatile bool         g_fault         = false;   // DRV8301 hardware fault latch
volatile bool         g_focReady      = false;   // FOCTask may drive (armed+calibrated)
static   bool         g_calibrated    = false;   // initFOC has succeeded once
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
    motor.enable();
    if (!g_calibrated) {
      if (motor.initFOC()) {
        g_calibrated = true;
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
        motor.controller        = MotionControlType::torque;
        motor.torque_controller = TorqueControlType::voltage; // Phase 4 -> foc_current
        g_active_target         = g_io.input_torque;          // Nm read as Uq(V) for now
        break;
    }
    if (g_io.vel_limit > 0.0f) motor.velocity_limit = g_io.vel_limit;
    if (g_io.pos_gain  > 0.0f) motor.P_angle.P      = g_io.pos_gain;

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
  g_io.pos_rev     = motor.shaft_angle / TWO_PI;
  g_io.vel_rev     = motor.shaft_velocity / TWO_PI;
  g_io.iq_setpoint = 0.0f;   // real values arrive with Phase 4 current sensing
  g_io.iq_measured = 0.0f;
  g_io.vbus        = readVbus();
  g_io.ibus        = 0.0f;
  g_io.cur_state   = g_focReady ? AXIS_CLOSED_LOOP : AXIS_IDLE;
}

// ============================================================================
//  FOCTask
// ============================================================================
static void FOCTask(void *) {
  g_focTask = xTaskGetCurrentTaskHandle();
  g_focTimer = new HardwareTimer(TIM6);          // free basic timer (TIM1=PWM, TIM3=enc)
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
  pinMode(PIN_M0_CS, OUTPUT); digitalWrite(PIN_M0_CS, HIGH);   // DRV SPI silent
  pinMode(PIN_M1_CS, OUTPUT); digitalWrite(PIN_M1_CS, HIGH);
  pinMode(PIN_N_FAULT, INPUT_PULLUP);
  analogReadResolution(12);

  // DRV8301 hardware reset, then leave it DISABLED (safe state).
  pinMode(PIN_EN_GATE, OUTPUT);
  digitalWrite(PIN_EN_GATE, LOW);  delay(50);
  digitalWrite(PIN_EN_GATE, HIGH); delay(50);
  digitalWrite(PIN_EN_GATE, LOW);                              // stay disabled

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- SimpleFOC + FreeRTOS + CANSimple ---");

  // ---- sensor: hardware timer encoder (no EXTI interrupts) ----
  encoder.init();
  motor.linkSensor(&encoder);

  // ---- driver (configured, NOT enabled) ----
  driver.voltage_power_supply = CFG_VBUS_NOMINAL;
  driver.pwm_frequency        = CFG_PWM_FREQ_HZ;
  driver.voltage_limit        = CFG_VOLT_LIMIT;
  if (!driver.init()) { Serial.println("[-] driver.init FAILED"); while (1); }
  motor.linkDriver(&driver);

  // ---- motor / control config (no calibration here) ----
  motor.voltage_limit        = CFG_VOLT_LIMIT;
  motor.velocity_limit       = CFG_VEL_LIMIT;
  motor.controller           = MotionControlType::torque;
  motor.torque_controller    = TorqueControlType::voltage;
  motor.foc_modulation       = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = CFG_VOLT_ALIGN;
  motor.motion_downsample    = MOTION_DOWNSAMPLE;
  motor.PID_velocity.P       = CFG_VEL_P;
  motor.PID_velocity.I       = CFG_VEL_I;
  motor.PID_velocity.D       = CFG_VEL_D;
  motor.PID_velocity.output_ramp = CFG_VEL_RAMP;
  motor.P_angle.P            = CFG_POS_P;
  motor.LPF_velocity.Tf      = CFG_LPF_VEL_TF;
  if (!motor.init()) { Serial.println("[-] motor.init FAILED"); while (1); }
  motor.disable();                                            // remain safe

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
  Serial.println("  CAN: Set_Axis_State(8)   or   serial: A");
  vTaskStartScheduler();
  for (;;) {}
}

void loop() {}
