// ============================================================================
//  configuration.h — the IEC 61131-3 CONFIGURATION (§2.7.1): the one place that
//  declares what tasks exist, what programs each one runs, and in what order
//  the hardware is brought up.
//
//  Adding a feature is meant to stop here: write a program, add it to a task's
//  program list in configuration.cpp. Nothing else in the firmware needs to
//  know it exists.
// ============================================================================
#pragma once

namespace configuration {

// Hardware bring-up, then program cold start, then the scheduler. Never returns.
void boot();

} // namespace configuration
