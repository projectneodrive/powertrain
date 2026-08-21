// ============================================================================
//  app.h — every control module's entry points, in one file.
//
//  A module is a .cpp under src/app/ plus two lines here. It owns its own state
//  (file-static, not globals), reads and writes state.h, and calls src/io/ for
//  anything touching hardware. It never creates a task and never blocks — with
//  one deliberate exception, marked below.
//
//  To add a feature: write src/app/yourthing.cpp, declare it here, and call
//  update() from a task in boot.cpp. That is the whole registration.
// ============================================================================
#pragma once

namespace app {

// 20 kHz, event-paced by TIM6. The hard real-time path — nothing here may
// block, allocate, or print.
namespace foc { void update(); }

// 1 kHz. The DC-bus protection ladder, the nFAULT latch and the brake chopper.
// Owns the fault latch outright: it both sets it and consumes clear requests.
namespace safety { void update(); }

// 1 kHz. The axis state machine: arm/disarm, gains, setpoint, watchdog.
//
// /!\ update() can BLOCK FOR SECONDS when it runs a commissioning sequence
// (hall calibration ~10 s, R/L characterisation). That is the one exception to
// "modules never block". It is tolerable only because this module runs in the
// COMMS task, below safety and FOC in priority, so the motor stays protected —
// but CAN RX is not drained for the duration and a master will see the node
// stop answering. Do not call sequences from a higher-priority task.
namespace control { void init(); void update(); }

// Blocking commissioning sequences, run from control on operator request. Hall
// calibration takes ~10 s; R/L characterisation a few seconds. Both drive the
// phases directly and require the motor DISARMED.
namespace calibration {
bool hallCalibrate();              // sweeps the 6 hall sectors, prints offsets
void characteriseMotor(bool safe); // measures phase R/L
}

// 1 kHz, split around control so a setpoint arriving this scan is acted on and
// reported in the same scan.
namespace comms {
void readFieldbus();       // drain CAN into the axis block
void publishTelemetry();   // publish the axis block + cyclic CAN TX
}

// 10 Hz. Serial command dispatch and the "t=..." telemetry line.
namespace console {
void update();
void printBanner();        // called once at boot, before the scheduler starts
}

}  // namespace app
