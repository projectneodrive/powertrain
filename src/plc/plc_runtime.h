// ============================================================================
//  plc_runtime.h — the RESOURCE (IEC 61131-3, §2.7.1): what actually executes
//  the task table on this MCU.
//
//  Boot is split in two calls so the CONFIGURATION can print its banner between
//  task creation and the scheduler start, exactly where it did before:
//
//      io::bringUp();                    // hardware, in its required order
//      plc::createTasks(TASKS, N);       // program init() + xTaskCreate
//      console::printBanner();           // still single-threaded here
//      plc::start();                     // never returns
// ============================================================================
#pragma once
#include <stddef.h>
#include "plc/plc_task.h"

namespace plc {

// Number of EVENT-triggered tasks the runtime can host. Each one costs a static
// task-handle slot and an ISR trampoline. Raise it if a second interrupt-paced
// task is ever added; createTasks() halts loudly if the table needs more.
#ifndef PLC_MAX_EVENT_TASKS
#define PLC_MAX_EVENT_TASKS 2
#endif

// Calls init() on every program in table order, then creates one FreeRTOS task
// per entry. Halts with a serial message if any xTaskCreate fails.
void createTasks(const TaskDef *tasks, size_t count);

// Starts the scheduler. Does not return.
void start();

} // namespace plc
