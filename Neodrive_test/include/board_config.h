// ============================================================================
//  board_config.h  —  Single source of truth for the ODrive v3.6 (MKS clone)
//  SimpleFOC firmware. Pins, motor presets, limits and RTOS timing live here.
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
//  CAN1 — used from Phase 6 onward
// ---------------------------------------------------------------------------
#define PIN_CAN_RX     PB8    // CAN1_RX
#define PIN_CAN_TX     PB9    // CAN1_TX

// ============================================================================
//  Motor / power presets.  Define MOTOR_PRESET in platformio.ini build_flags
//  to override (e.g. -D MOTOR_PRESET=MOTOR_PRESET_EBIKE).
// ============================================================================
#define MOTOR_PRESET_BENCH   1   // 7pp drone motor + 600 PPR quadrature encoder
#define MOTOR_PRESET_EBIKE   2   // 25pp hub motor + hall sensors

#ifndef MOTOR_PRESET
#define MOTOR_PRESET MOTOR_PRESET_BENCH
#endif

#if MOTOR_PRESET == MOTOR_PRESET_BENCH
  #define CFG_POLE_PAIRS   7
  #define CFG_ENC_PPR      600      // quadrature 600 PPR -> 2400 CPR
  #define CFG_KV           190.0f   // drone motor KV
#elif MOTOR_PRESET == MOTOR_PRESET_EBIKE
  #define CFG_POLE_PAIRS   25
  #define CFG_ENC_PPR      600      // (only used if a quadrature enc is fitted)
  #define CFG_KV           9.38f    // hub motor KV
#else
  #error "Unknown MOTOR_PRESET"
#endif

// Torque constant (Nm/A). Kt = 8.27 / KV (same relation ODrive uses).
#define CFG_KT           (8.27f / CFG_KV)

// ---------------------------------------------------------------------------
//  Power / limits — conservative values for bring-up. Tighten per motor.
// ---------------------------------------------------------------------------
#define CFG_VBUS_NOMINAL   24.0f    // driver.voltage_power_supply
#define CFG_PWM_FREQ_HZ    20000    // 20 kHz (matches FOC tick; keeps sense window sane)
#define CFG_VOLT_LIMIT     2.0f     // motor/driver voltage limit (safety)
#define CFG_VOLT_ALIGN     0.8f     // voltage used during initFOC alignment
#define CFG_CURRENT_LIMIT  15.0f    // A (used once current sensing is enabled)
#define CFG_VEL_LIMIT      100.0f   // rad/s

// ---------------------------------------------------------------------------
//  Current-sense hardware — VERIFY THESE ON YOUR CLONE (silkscreen/schematic).
//  Shunt value and DRV8301 amp gain directly scale measured phase current.
//  The DRV_GAIN here MUST equal the gain programmed into DRV8301 CTRL2.
// ---------------------------------------------------------------------------
#define CFG_SHUNT_OHMS     0.0005f  // 0.5 mOhm (ODrive 56V); clones vary (verify!)
#define CFG_DRV_GAIN       40.0f    // V/V  (DRV8301: 10/20/40/80 selectable)

// ============================================================================
//  FreeRTOS timing / priorities  (higher number = higher urgency)
// ============================================================================
#define FOC_TICK_HZ        20000    // FOC loop rate (TIM6 -> FOCTask notify)
#define MOTION_DOWNSAMPLE  20       // move() runs at FOC_TICK_HZ/DOWNSAMPLE = 1 kHz

#define PRIO_SAFETY        5        // top: fault latch / watchdog
#define PRIO_FOC           4        // FOC loop
#define PRIO_CAN           3        // CAN RX drain (Phase 6)
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
#define STACK_SAFETY     256
#define STACK_TELEMETRY  512
