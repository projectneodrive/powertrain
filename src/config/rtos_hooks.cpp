// ============================================================================
//  rtos_hooks.cpp — FreeRTOS failure hooks.
//
//  Enabled by configCHECK_FOR_STACK_OVERFLOW / configUSE_MALLOC_FAILED_HOOK in
//  include/STM32FreeRTOSConfig.h. Both failure modes otherwise produce a silent
//  hang with no serial output at all; these make the failure self-report, and
//  cut the gate driver, since RTOS state can no longer be trusted to keep
//  running PRG_FOC and PRG_SAFETY correctly.
//
//  /!\ The symbol names below are the ones the kernel declares in tasks.c and
//  links against — vApplicationStackOverflowHook, NOT ...Handler. Getting that
//  name wrong does not fail loudly: the definition simply sits there unused
//  while the kernel looks for the real symbol.
// ============================================================================
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "io/io_brake.h"
#include "io/io_gate.h"

extern "C" void vApplicationStackOverflowHook(TaskHandle_t /*xTask*/, char *pcTaskName) {
  io::brake::off();
  io::gate::disable();
  Serial.print("\n[FATAL] Stack overflow in task \"");
  Serial.print(pcTaskName);
  Serial.println("\" -- halting. Increase its STACK_* in config/plc_config.h.");
  Serial.flush();
  for (;;) {}
}

extern "C" void vApplicationMallocFailedHook(void) {
  io::brake::off();
  io::gate::disable();
  Serial.println("\n[FATAL] FreeRTOS heap allocation failed (configTOTAL_HEAP_SIZE exhausted) -- halting.");
  Serial.flush();
  for (;;) {}
}
