// ============================================================================
//  plc_config.h  —  IEC 61131-3 CONFIGURATION parameters: task rates, priorities
//  and stack depths for the PLC runtime (src/plc/), plus the interrupt priority
//  policy. Nothing about the motor or the board wiring lives here.
//
//  The TASKS[] table itself is in src/config/configuration.cpp; this file holds
//  the numbers it is built from.
// ============================================================================
#pragma once

// ============================================================================
//  Task timing / priorities  (higher number = higher urgency)
// ============================================================================
#define FOC_TICK_HZ        20000    // FOC loop rate (TIM6 event -> PRG_FOC)
#define MOTION_DOWNSAMPLE  20       // move() runs at FOC_TICK_HZ/DOWNSAMPLE = 1 kHz

#define PRIO_SAFETY        5        // top: fault latch / watchdog
#define PRIO_FOC           4        // FOC loop
#define PRIO_CAN           3        // CAN RX drain
#define PRIO_COMMS         PRIO_CAN // alias: CAN + control-bridge task
#define PRIO_TELEMETRY     2        // telemetry / debug

// Task scan intervals (ms) for the cyclic tasks.
#define SCAN_MS_SAFETY     1
#define SCAN_MS_COMMS      1
#define SCAN_MS_CONSOLE    100

// NVIC preemption priority for any ISR that calls a FreeRTOS *FromISR API.
// STM32duino FreeRTOS: configMAX_SYSCALL_INTERRUPT_PRIORITY derives from
// library value 5, so such ISRs must sit at a NUMERICALLY >= 5 priority
// (i.e. less urgent). 6 gives margin. The current-sense ADC ISR (no FreeRTOS
// call) may stay more urgent.
#define NVIC_PRIO_RTOS_SAFE  6

// Task stack depths (in WORDS = 4 bytes). Kept modest to fit the default
// FreeRTOS heap; bump if xTaskCreate returns pdFAIL.
#define STACK_FOC        768
#define STACK_SAFETY     512   // PRG_SAFETY: HAL ADC + Serial (fault message)
#define STACK_TELEMETRY  512
#define STACK_COMMS      768
