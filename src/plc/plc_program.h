// ============================================================================
//  plc_program.h — the PROGRAM POU (IEC 61131-3, §2.5.3).
//
//  A Program is a unit of logic that runs cyclically. It owns its function
//  block instances as members (so their state is scoped to it, not global),
//  reads and writes the process image (gvl/gvl.h), and does nothing else:
//
//    - it never creates a task            -> that is the CONFIGURATION's job
//                                            (src/config/configuration.cpp)
//    - it never touches a peripheral      -> that is the I/O layer's job (src/io/)
//    - it never blocks on a delay         -> the task's trigger paces it
//
//  init() runs ONCE, before the scheduler starts, in the order the programs
//  appear in the task table. scan() runs once per task cycle. Neither may
//  assume anything about the other tasks' state beyond what the GVL exposes.
//
//  Programs are declared as file-scope singletons in their own translation unit
//  and referenced from the task table, e.g.:
//
//      // prog_safety.cpp
//      PrgSafety prgSafety;
//      // configuration.cpp
//      static plc::Program* const SAFE_PROGRAMS[] = { &prgSafety };
// ============================================================================
#pragma once
#include <stdint.h>   // programs routinely hold uintN_t scan counters

namespace plc {

class Program {
 public:
  virtual ~Program() {}

  // Cold start. Called once, before vTaskStartScheduler(), in task-table order.
  // Safe to use Serial here; the RTOS is not running yet.
  virtual void init() {}

  // One cycle. Called by the task the program is bound to.
  virtual void scan() = 0;

  // For diagnostics/fault messages.
  virtual const char* name() const = 0;
};

} // namespace plc
