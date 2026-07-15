/*
 * STM32FreeRTOSConfig.h — project override of the STM32duino FreeRTOS defaults.
 *
 * Picked up automatically: FreeRTOSConfig.h (in the STM32duino FreeRTOS
 * library) does `#if __has_include("STM32FreeRTOSConfig.h")` and, if found,
 * includes ONLY this file instead of FreeRTOSConfig_Default.h. This is a
 * full copy of that default with two flags flipped:
 *   - configCHECK_FOR_STACK_OVERFLOW: 0 -> 2 (task stack overflow detection)
 *   - configUSE_MALLOC_FAILED_HOOK:   0 -> 1 (heap exhaustion detection)
 * Both failure modes previously produced a silent hang with zero serial
 * output (e.g. right after "Serial cmds: ..." in setup(), before any task
 * telemetry ever printed) since neither the default kernel build nor the
 * app defined a handler to report them. See vApplicationStackOverflowHandler
 * / vApplicationMallocFailedHook in main.cpp for what happens when these
 * fire.
 */

#ifndef STM32_FREERTOS_CONFIG_H
#define STM32_FREERTOS_CONFIG_H

/* Set to 1 to use default blink hook if configUSE_MALLOC_FAILED_HOOK is 1 */
#ifndef configUSE_MALLOC_FAILED_HOOK_BLINK
  #define configUSE_MALLOC_FAILED_HOOK_BLINK  0
#endif
/* Set to 1 to used default blink if configCHECK_FOR_STACK_OVERFLOW is 1 or 2 */
#ifndef configCHECK_FOR_STACK_OVERFLOW_BLINK
  #define configCHECK_FOR_STACK_OVERFLOW_BLINK 0
#endif

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
 #include <stdint.h>
 extern uint32_t SystemCoreClock;
#endif

#define configMAX_PRIORITIES              (7)

extern char _end; /* Defined in the linker script */
extern char _estack; /* Defined in the linker script */
extern char _Min_Stack_Size; /* Defined in the linker script */
#ifndef configMINIMAL_STACK_SIZE
#define configMINIMAL_STACK_SIZE          ((uint16_t)((uint32_t)&_Min_Stack_Size/8))
#endif
#ifndef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE             ((size_t)((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size - (uint32_t)&_end))
#endif
#ifndef configISR_STACK_SIZE_WORDS
#define configISR_STACK_SIZE_WORDS        ((uint32_t)&_Min_Stack_Size/4)
#endif

#define configUSE_PREEMPTION              1
#define configUSE_IDLE_HOOK               1
#define configUSE_TICK_HOOK               1
#define configCPU_CLOCK_HZ                (SystemCoreClock)
#define configTICK_RATE_HZ                ((TickType_t)1000)
#define configMAX_TASK_NAME_LEN           (16)
#define configUSE_TRACE_FACILITY          1
#define configUSE_16_BIT_TICKS            0
#define configIDLE_SHOULD_YIELD           1
#define configUSE_MUTEXES                 1
#define configQUEUE_REGISTRY_SIZE         8
#define configCHECK_FOR_STACK_OVERFLOW    2
#define configUSE_RECURSIVE_MUTEXES       1
#define configUSE_MALLOC_FAILED_HOOK      1
#define configUSE_APPLICATION_TASK_TAG    0
#define configUSE_COUNTING_SEMAPHORES     1
#define configGENERATE_RUN_TIME_STATS     0
#define configUSE_NEWLIB_REENTRANT        1

#define configENABLE_MPU                  0
#define configENABLE_FPU                  1
#define configENABLE_TRUSTZONE            0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES             0
#define configMAX_CO_ROUTINE_PRIORITIES  (2)

/* Software timer definitions. */
#define configUSE_TIMERS                  1
#define configTIMER_TASK_PRIORITY        (2)
#define configTIMER_QUEUE_LENGTH         10
#define configTIMER_TASK_STACK_DEPTH    (configMINIMAL_STACK_SIZE * 2)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet          1
#define INCLUDE_uxTaskPriorityGet         1
#define INCLUDE_vTaskDelete               1
#define INCLUDE_vTaskCleanUpResources     1
#define INCLUDE_vTaskSuspend              1
#define INCLUDE_vTaskDelayUntil           1
#define INCLUDE_vTaskDelay                1
#define INCLUDE_xTaskGetSchedulerState    1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle    1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 #define configPRIO_BITS                  __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS                  4        /* 15 priority levels */
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   0xf
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY   ( 14 << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
   standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

#endif /* STM32_FREERTOS_CONFIG_H */
