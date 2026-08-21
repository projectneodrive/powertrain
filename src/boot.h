// ============================================================================
//  boot.h — bring the hardware up, start the four tasks, hand over to FreeRTOS.
//
//  This is the one file that knows the whole system: what hardware exists and
//  in what order it must be initialised, what tasks run, how fast, at what
//  priority, and which modules each one calls. Everything else is a module that
//  knows only its own job.
//
//  Adding a feature stops here: write a module with init()/update() under
//  src/app/, then add one call to the task that should run it.
// ============================================================================
#pragma once

namespace boot {

// Hardware, then module cold start, then the scheduler. Never returns.
void run();

}  // namespace boot
