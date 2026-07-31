// ============================================================================
//  SimpleFOC + FreeRTOS + ODrive CANSimple  —  ODrive v3.6 (MKS clone) / F405
//
//  The firmware is organised as an IEC 61131-3 style PLC. Where to look:
//
//    src/config/configuration.cpp   the CONFIGURATION: the task table and the
//                                   hardware boot order. Start here.
//    src/plc/                       the runtime: Program, Task, the scheduler
//                                   binding, and the standard function blocks
//    include/gvl/                   VAR_GLOBAL — the process image, and the
//                                   single-writer rules that make it safe
//    src/prog/                      the PROGRAMs: FOC, safety, control, comms,
//                                   console
//    src/fb/                        application FUNCTION_BLOCKs
//    src/io/                        the only code that touches hardware
//    src/seq/                       blocking commissioning sequences
//    include/config/                pins, motor/power tuning, task timing
//    include/console_commands.h     the serial command set (one line each)
//    include/can_commands.h         the CANSimple command set (one line each)
//    include/telemetry_schema.h     the streamed channels, shared with the GUI
//
//  setup() therefore has nothing left to do but hand over to the configuration.
// ============================================================================
#include <Arduino.h>
#include "config/configuration.h"

void setup() {
  configuration::boot();   // does not return
}

void loop() {}
