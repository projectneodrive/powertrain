// ============================================================================
//  plc_runtime.cpp — see plc_runtime.h.
//
//  Both task shapes reproduce the hand-written bodies this replaced, including
//  the details that are not free to reorder:
//
//   * CYCLIC: programs run FIRST, then vTaskDelayUntil against a tick baseline
//     captured before the loop. Delaying "until" an absolute baseline is what
//     keeps a 1 kHz task at 1 kHz regardless of how long a scan took; a plain
//     vTaskDelay would let the period drift by the execution time.
//
//   * EVENT: the task handle is published BEFORE the timer is created and
//     resumed. The ISR fires as soon as the timer runs, so if the handle were
//     published afterwards there would be a window where the notification is
//     dropped. Publishing it first also means the very first tick is already
//     counted.
//     The notification is deliberately NOT a queue: ulTaskNotifyTake(pdTRUE,..)
//     clears the counter, so a tick missed while the task was preempted is lost,
//     not made up later. That is the correct behaviour for a control loop — one
//     late execution is better than two back-to-back ones with a bogus dt — but
//     it is also why anything that steals CPU from an event task at a fixed
//     rate shows up as a periodic disturbance rather than as jitter.
// ============================================================================
#include "plc/plc_runtime.h"
#include "config/plc_config.h"

namespace plc {
namespace {

// ---- Event-task plumbing ---------------------------------------------------
// One static handle slot per event task. The ISR must reach its task through a
// plain pointer read: a capturing lambda would go through std::function, which
// heap-allocates and adds an indirection inside a 20 kHz interrupt.
TaskHandle_t s_eventHandle[PLC_MAX_EVENT_TASKS] = {};

template <int SLOT>
void eventTrampoline() {
  BaseType_t hpw = pdFALSE;
  if (s_eventHandle[SLOT]) vTaskNotifyGiveFromISR(s_eventHandle[SLOT], &hpw);
  portYIELD_FROM_ISR(hpw);
}

void (*const TRAMPOLINE[PLC_MAX_EVENT_TASKS])() = {
  eventTrampoline<0>,
#if PLC_MAX_EVENT_TASKS > 1
  eventTrampoline<1>,
#endif
};

// Per-task context handed to the body as the FreeRTOS task parameter. Static
// storage: the task outlives createTasks()'s stack frame.
struct TaskCtx {
  const TaskDef *def;
  uint8_t        event_slot;
};

TaskCtx s_ctx[8];        // enough for the current table; checked in createTasks
uint8_t s_ctx_used = 0;

// ---- Fatal boot failure ----------------------------------------------------
// Reached only before the scheduler exists, so a busy loop is the right answer:
// there is nothing left to schedule and the power stage was never armed.
void fatal(const char *msg) {
  Serial.print("[-] ");
  Serial.println(msg);
  Serial.flush();
  for (;;) {}
}

// ---- The task body ---------------------------------------------------------
void taskBody(void *arg) {
  const TaskCtx &ctx = *(const TaskCtx *)arg;
  const TaskDef &t   = *ctx.def;

  if (t.trigger.kind == Trigger::EVENT) {
    s_eventHandle[ctx.event_slot] = xTaskGetCurrentTaskHandle();

    HardwareTimer *tim = new HardwareTimer(t.trigger.timer);
    tim->setOverflow(t.trigger.event_hz, HERTZ_FORMAT);
    tim->attachInterrupt(TRAMPOLINE[ctx.event_slot]);
    // The ISR calls a FreeRTOS *FromISR API, so it must sit at a numerically
    // >= configMAX_SYSCALL_INTERRUPT_PRIORITY priority. See plc_config.h.
    tim->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);
    tim->resume();

    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      for (uint8_t i = 0; i < t.program_count; i++) t.programs[i]->scan();
    }
  }

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    for (uint8_t i = 0; i < t.program_count; i++) t.programs[i]->scan();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(t.trigger.interval_ms));
  }
}

} // namespace

// ---------------------------------------------------------------------------
void createTasks(const TaskDef *tasks, size_t count) {
  if (count > (sizeof(s_ctx) / sizeof(s_ctx[0])))
    fatal("PLC task table larger than s_ctx[] (plc_runtime.cpp)");

  // Cold start every program, in table order, while still single-threaded.
  for (size_t t = 0; t < count; t++)
    for (uint8_t p = 0; p < tasks[t].program_count; p++)
      tasks[t].programs[p]->init();

  uint8_t next_event_slot = 0;
  for (size_t t = 0; t < count; t++) {
    const TaskDef &def = tasks[t];

    uint8_t slot = 0;
    if (def.trigger.kind == Trigger::EVENT) {
      if (next_event_slot >= PLC_MAX_EVENT_TASKS)
        fatal("more EVENT tasks than PLC_MAX_EVENT_TASKS (plc_runtime.h)");
      slot = next_event_slot++;
    }

    s_ctx[s_ctx_used] = TaskCtx{&def, slot};
    BaseType_t r = xTaskCreate(taskBody, def.name, def.stack_words,
                               &s_ctx[s_ctx_used], def.priority, NULL);
    s_ctx_used++;
    if (r != pdPASS) fatal("xTaskCreate FAILED (Check FreeRTOS heap)");
  }
}

void start() {
  vTaskStartScheduler();
  for (;;) {}
}

} // namespace plc
