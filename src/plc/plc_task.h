// ============================================================================
//  plc_task.h — the TASK declaration (IEC 61131-3, §2.7.2).
//
//  A task binds an ordered list of programs to an execution trigger, a priority
//  and a stack. The CONFIGURATION (src/config/configuration.cpp) declares them
//  all in one table; plc_runtime.cpp turns each entry into one FreeRTOS task.
//
//  Two trigger kinds, matching what the hardware actually offers:
//
//    Cyclic(ms)          periodic, paced by vTaskDelayUntil -> no drift
//    Event(TIMx, hz)     the task blocks until an interrupt notifies it. The
//                        runtime owns the timer and its ISR; the program never
//                        sees them. Used for the 20 kHz FOC loop, where a
//                        1 ms-resolution RTOS tick is useless.
//
//  Programs in a task run in DECLARED ORDER, once per trigger. That ordering is
//  the contract: it is how "read inputs -> run logic -> write outputs" is
//  expressed, e.g. the COMMS task running
//      PRG_FIELDBUS_IN -> PRG_CONTROL -> PRG_TELEMETRY_OUT
//  so a CAN setpoint that arrives this cycle is acted on this cycle, and the
//  telemetry published reflects the result rather than the previous cycle's.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "plc/plc_program.h"

namespace plc {

struct Trigger {
  enum Kind : uint8_t { CYCLIC, EVENT };

  Kind         kind;
  uint32_t     interval_ms;   // CYCLIC only
  TIM_TypeDef *timer;         // EVENT only — owned by the runtime
  uint32_t     event_hz;      // EVENT only
};

inline Trigger Cyclic(uint32_t interval_ms) {
  return Trigger{Trigger::CYCLIC, interval_ms, nullptr, 0};
}

inline Trigger Event(TIM_TypeDef *timer, uint32_t hz) {
  return Trigger{Trigger::EVENT, 0, timer, hz};
}

struct TaskDef {
  const char      *name;           // FreeRTOS task name (kept short: shows in traces)
  Trigger          trigger;
  UBaseType_t      priority;
  uint16_t         stack_words;    // depth in WORDS (4 bytes), as xTaskCreate wants
  Program *const  *programs;
  uint8_t          program_count;
};

// Build a TaskDef from a program array, deducing the count so the table can
// never disagree with the array it points at.
template <size_t N>
constexpr TaskDef Task(const char *name, Trigger trigger, UBaseType_t priority,
                       uint16_t stack_words, Program *const (&programs)[N]) {
  return TaskDef{name, trigger, priority, stack_words, programs, (uint8_t)N};
}

} // namespace plc
