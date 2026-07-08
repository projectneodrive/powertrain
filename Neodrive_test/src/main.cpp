// ============================================================================
//  SimpleFOC + FreeRTOS firmware  —  ODrive v3.6 (MKS clone) / STM32F405
//
//  Phase 1-3 milestone:
//   * Phase 1: all pins/limits centralized in include/board_config.h
//   * Phase 2: quadrature sensing via STM32HWEncoder (TIM3 hardware counter)
//              -> replaces SimpleFOC's software Encoder + enableInterrupts().
//              This DELETES the ~100 kHz A/B EXTI ISRs that were starving the
//              FreeRTOS scheduler. The hardware timer counts edges for free.
//   * Phase 3: FreeRTOS task layout:
//              - FOCTask  (prio 4): woken by a 20 kHz TIM6 tick, runs loopFOC()
//                every tick and move() every 20th (1 kHz) via motion_downsample.
//              - SafetyTask (prio 5): polls DRV8301 nFAULT, hardware-cuts EN_GATE.
//              - TelemetryTask (prio 2): steady-cadence debug + serial commands.
//
//  Torque is still voltage-mode here (no current sensing yet — that is Phase 4).
//  Serial commands (115200 USB-CDC):  T<volts>  set target voltage
//                                      C         clear latched fault
// ============================================================================
#include <Arduino.h>
#include <SimpleFOC.h>
#include <STM32FreeRTOS.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"   // SimpleFOCDrivers
#include "board_config.h"

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
//  SimpleFOC objects
// ============================================================================
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_M0_INH_A, PIN_M0_INL_A,
                                       PIN_M0_INH_B, PIN_M0_INL_B,
                                       PIN_M0_INH_C, PIN_M0_INL_C, PIN_EN_GATE);
BLDCMotor      motor  = BLDCMotor(CFG_POLE_PAIRS);

// Hardware quadrature encoder on TIM3 (PB4/PB5). No software interrupts.
STM32HWEncoder encoder = STM32HWEncoder(CFG_ENC_PPR, PIN_ENC_A, PIN_ENC_B);

// ============================================================================
//  Shared state (single 32-bit values -> atomic on Cortex-M4)
// ============================================================================
// Hard-coded open test torque (voltage mode), matching the pre-RTOS main.cpp:
// the motor starts turning right after calibration. Override live with 'T<volts>'.
volatile float        g_target_voltage = 0.8f;   // commanded Uq (voltage mode)
volatile bool         g_fault          = false;   // latched fault
static   TaskHandle_t g_focTask        = nullptr; // notified by the TIM6 ISR
static   HardwareTimer *g_focTimer     = nullptr;

// ============================================================================
//  20 kHz FOC tick ISR -> wake FOCTask
// ============================================================================
static void onFocTick() {
  BaseType_t hpw = pdFALSE;
  if (g_focTask) vTaskNotifyGiveFromISR(g_focTask, &hpw);
  portYIELD_FROM_ISR(hpw);
}

// ============================================================================
//  FOCTask — the real-time control loop
// ============================================================================
static void FOCTask(void *) {
  g_focTask = xTaskGetCurrentTaskHandle();

  // Configure the FOC time base. TIM6 is a free basic timer (TIM1 = PWM,
  // TIM3 = encoder). Its ISR only notifies FreeRTOS, so it MUST sit at a
  // syscall-safe NVIC priority.
  g_focTimer = new HardwareTimer(TIM6);
  g_focTimer->setOverflow(FOC_TICK_HZ, HERTZ_FORMAT);
  g_focTimer->attachInterrupt(onFocTick);
  g_focTimer->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);
  g_focTimer->resume();

  for (;;) {
    // Block until the next 20 kHz tick — this is what frees the CPU for the
    // lower-priority tasks (the old taskYIELD() busy-loop never blocked).
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (g_fault) continue;

    motor.loopFOC();               // reads HW encoder + commutates (20 kHz)
    motor.move(g_target_voltage);  // motion_downsample -> effective 1 kHz
  }
}

// ============================================================================
//  SafetyTask — hardware fault backstop (highest priority)
// ============================================================================
static void SafetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (digitalRead(PIN_N_FAULT) == LOW) {
      // Definitive, race-free hardware cut, independent of the FOC loop.
      digitalWrite(PIN_EN_GATE, LOW);
      g_fault = true;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

// ============================================================================
//  TelemetryTask — steady-cadence debug (proves the scheduler is not starved)
//  and a tiny serial command parser.
// ============================================================================
static void handleSerial() {
  static char buf[24];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf[idx] = '\0';
      if (idx > 0) {
        if (buf[0] == 'T' || buf[0] == 't') {
          g_target_voltage = atof(buf + 1);
        } else if (buf[0] == 'C' || buf[0] == 'c') {
          if (digitalRead(PIN_N_FAULT) == HIGH) {  // only if fault cleared
            g_target_voltage = 0.0f;
            digitalWrite(PIN_EN_GATE, HIGH);
            g_fault = false;
          }
        }
      }
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

static void TelemetryTask(void *) {
  uint32_t beat = 0;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    handleSerial();
    Serial.print("t=");    Serial.print(millis());
    Serial.print(" #");    Serial.print(beat++);
    Serial.print(" ang="); Serial.print(motor.shaft_angle, 3);
    Serial.print(" vel="); Serial.print(motor.shaft_velocity, 2);
    Serial.print(" Uq=");  Serial.print(g_target_voltage, 2);
    Serial.println(g_fault ? " [FAULT]" : " [OK]");
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));   // fixed 10 Hz cadence
  }
}

// ============================================================================
//  setup — bring-up + calibration BEFORE the scheduler starts
//  (delay()/millis() run off the HAL SysTick until vTaskStartScheduler()).
// ============================================================================
void setup() {
  pinMode(PIN_M0_CS, OUTPUT); digitalWrite(PIN_M0_CS, HIGH);   // DRV SPI silent
  pinMode(PIN_M1_CS, OUTPUT); digitalWrite(PIN_M1_CS, HIGH);
  pinMode(PIN_N_FAULT, INPUT_PULLUP);

  // DRV8301 hardware reset (LOW -> HIGH)
  pinMode(PIN_EN_GATE, OUTPUT);
  digitalWrite(PIN_EN_GATE, LOW);  delay(50);
  digitalWrite(PIN_EN_GATE, HIGH); delay(50);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- SimpleFOC + FreeRTOS (STM32HWEncoder) ---");

  // ---- sensor: hardware timer encoder (no EXTI interrupts) ----
  encoder.init();
  motor.linkSensor(&encoder);

  // ---- driver ----
  driver.voltage_power_supply = CFG_VBUS_NOMINAL;
  driver.pwm_frequency        = CFG_PWM_FREQ_HZ;
  driver.voltage_limit        = CFG_VOLT_LIMIT;
  if (!driver.init()) { Serial.println("[-] driver.init FAILED"); while (1); }
  motor.linkDriver(&driver);

  // ---- motor / control ----
  motor.voltage_limit        = CFG_VOLT_LIMIT;
  motor.velocity_limit       = CFG_VEL_LIMIT;
  motor.controller           = MotionControlType::torque;
  motor.torque_controller    = TorqueControlType::voltage;
  motor.foc_modulation       = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = CFG_VOLT_ALIGN;
  motor.motion_downsample    = MOTION_DOWNSAMPLE;
  if (!motor.init()) { Serial.println("[-] motor.init FAILED"); while (1); }

  driver.enable();
  motor.enable();

  Serial.println("Calibrating (initFOC)... keep the motor free.");
  if (!motor.initFOC()) { Serial.println("[-] initFOC FAILED"); while (1); }
  Serial.print("initFOC OK | dir=");
  Serial.print(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial.print(" zero_elec=");
  Serial.println(motor.zero_electric_angle, 4);

  // ---- launch FreeRTOS ----
  BaseType_t r1 = xTaskCreate(SafetyTask,    "SAFE", STACK_SAFETY,    NULL, PRIO_SAFETY,    NULL);
  BaseType_t r2 = xTaskCreate(FOCTask,       "FOC",  STACK_FOC,       NULL, PRIO_FOC,       NULL);
  BaseType_t r3 = xTaskCreate(TelemetryTask, "TEL",  STACK_TELEMETRY, NULL, PRIO_TELEMETRY, NULL);
  if (r1 != pdPASS || r2 != pdPASS || r3 != pdPASS) {
    Serial.println("[-] xTaskCreate FAILED (bump FreeRTOS heap)");
    while (1);
  }

  Serial.print("Scheduler starting @ Uq="); Serial.print(g_target_voltage, 2);
  Serial.println(" V. Commands: T<volts>, C(clear fault).");
  vTaskStartScheduler();

  // never reached
  for (;;) {}
}

void loop() {}   // unused under FreeRTOS
