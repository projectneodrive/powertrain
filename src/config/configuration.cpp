// ============================================================================
//  configuration.cpp — the CONFIGURATION: the task table and the boot order.
// ============================================================================
#include "config/configuration.h"
#include "plc/plc_runtime.h"
#include "prog/prog_foc.h"
#include "prog/prog_safety.h"
#include "prog/prog_control.h"
#include "prog/prog_comms.h"
#include "prog/prog_console.h"
#include "io/io_gate.h"
#include "io/io_brake.h"
#include "io/io_motor.h"
#include "io/io_vbus.h"
#include "io/io_can.h"
#include "config/plc_config.h"
#include "config/motor_config.h"

namespace configuration {
namespace {

// ---------------------------------------------------------------------------
//  RESOURCE: the task table.
//
//  Programs inside a task run in the order listed, once per trigger. For COMMS
//  that order IS the scan cycle — read the fieldbus, run the state machine,
//  publish — so a setpoint that arrives in a given millisecond is acted on and
//  reported in that same millisecond.
//
//  Names, priorities and stack depths come from config/plc_config.h.
// ---------------------------------------------------------------------------
plc::Program* const SAFE_PROGRAMS[] = {
  &prog::prgSafety,
};
plc::Program* const FOC_PROGRAMS[] = {
  &prog::prgFoc,
};
plc::Program* const COMMS_PROGRAMS[] = {
  &prog::prgFieldbusIn,
  &prog::prgControl,
  &prog::prgTelemetryOut,
};
plc::Program* const CONSOLE_PROGRAMS[] = {
  &prog::prgConsole,
};

const plc::TaskDef TASKS[] = {
  plc::Task("SAFE",  plc::Cyclic(SCAN_MS_SAFETY),        PRIO_SAFETY,    STACK_SAFETY,    SAFE_PROGRAMS),
  plc::Task("FOC",   plc::Event(TIM6, FOC_TICK_HZ),      PRIO_FOC,       STACK_FOC,       FOC_PROGRAMS),
  plc::Task("COMMS", plc::Cyclic(SCAN_MS_COMMS),         PRIO_COMMS,     STACK_COMMS,     COMMS_PROGRAMS),
  plc::Task("SER",   plc::Cyclic(SCAN_MS_CONSOLE),       PRIO_TELEMETRY, STACK_TELEMETRY, CONSOLE_PROGRAMS),
};
constexpr size_t TASK_COUNT = sizeof(TASKS) / sizeof(TASKS[0]);

// ---------------------------------------------------------------------------
//  Hardware bring-up. The ORDER here is load-bearing; see the notes inline.
// ---------------------------------------------------------------------------
void bringUpHardware() {
  // AUX (brake) half-bridge: gates LOW immediately, BEFORE anything switches
  // those pins to an alternate function, so the half-bridge never passes
  // through an undefined state at power-up. io::brake::init() takes over later.
  io::brake::preInit();

  // Gate driver GPIO + the DRV8301 reset pulse. Before Serial: the DRV needs
  // its 50 ms settling either way, and doing it first means the power stage is
  // in a known state as early as possible.
  io::gate::preInit();

  // Board-global ADC setting, not owned by any one module: both the current
  // sense and io_vbus assume 12-bit conversions.
  analogReadResolution(12);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }
  SimpleFOCDebug::enable(&Serial);
  Serial.println("\n--- SimpleFOC + FreeRTOS + CANSimple ---");

  io::gate::init();     // DRV8301 over SPI: out of reset, amplifier gain
  io::motor::init();    // sensor, driver, current sense, motor parameters

  // Brake chopper (leaves init() STOPPED) + the Vbus ADC configuration, both
  // BEFORE PRG_SAFETY starts reading them.
  // io::brake::init() runs after the DRV8301 reset above: GVDD, which powers
  // the LM5109B, only exists once the DRV8301 is awake.
  io::brake::init();
  // Must follow io::motor::init(): it picks whichever ADC the current sense did
  // not take, so the current sense has to have claimed one first.
  io::vbus::init();

  Serial.print("Brake resistor: "); Serial.print(CFG_BRAKE_R, 1);
#if CFG_BUS_SOURCE == CFG_BUS_SOURCE_PSU
  Serial.print(" ohm on AUX [PSU], chopper ");
#else
  Serial.print(" ohm on AUX [BATTERY], chopper ");
#endif
  Serial.print(CFG_BRAKE_VBUS_OFF, 1);
  Serial.print("/");                  Serial.print(CFG_BRAKE_VBUS_ON, 1);
  Serial.print(" V gain ");           Serial.print(CFG_BRAKE_GAIN, 2);
  Serial.print(", regen derate ");    Serial.print(CFG_VBUS_REGEN_START, 1);
  Serial.print("-");                  Serial.print(CFG_VBUS_REGEN_FULL, 1);
  Serial.print(" V, OV trip ");       Serial.print(CFG_VBUS_OV_TRIP, 1);
  Serial.println(" V");

  io::can::init();
}

} // namespace

// ---------------------------------------------------------------------------
void boot() {
  bringUpHardware();

  // Cold-starts every program (which is where the axis defaults are seeded from
  // the configuration), then creates one FreeRTOS task per table entry.
  plc::createTasks(TASKS, TASK_COUNT);

  Serial.println("SAFE state (disarmed). Send 'A' via serial or CAN CLOSED_LOOP state to arm.");
  prog::printConsoleBanner();

  plc::start();   // does not return
}

} // namespace configuration
