// ============================================================================
//  main.cpp — the entry point, and nothing else.
//
//  Everything the firmware does lives in modules:
//
//    src/boot.cpp    hardware bring-up order, the four tasks, what each runs
//    src/app/        the control modules: foc, safety, control, comms, console
//    src/io/         the only code that touches a register, pin or peripheral
//    src/state.h     the values modules share, and who is allowed to write them
//    src/util/       small stateful helpers (debounce, hysteresis, clamp)
//
//  Configuration is in include/config/. The command, telemetry and CAN tables
//  are in include/ and are compiled by the host tooling too — see src/README.md.
// ============================================================================
#include <Arduino.h>   // declares setup()/loop() with the linkage the core expects
#include "boot.h"

void setup() { boot::run(); }   // does not return

void loop() {}
