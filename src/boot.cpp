// ============================================================================
//  boot.cpp — see boot.h.
//
//  The four tasks are written out as four functions rather than built from a
//  table. There are four of them, three run exactly one module, and the one
//  that runs three expresses its ordering as three statements you can read.
//
//  THREE DETAILS BELOW ARE LOAD-BEARING AND NOT OBVIOUS. All three compile
//  perfectly if you get them wrong, and all three fail only at run time:
//
//   1. The FOC task publishes its own handle BEFORE starting the timer. The ISR
//      fires as soon as the timer runs; publish afterwards and there is a window
//      where notifications are dropped.
//   2. The timer ISR sits at NVIC_PRIO_RTOS_SAFE. It calls a FreeRTOS *FromISR
//      API, so it must be NUMERICALLY >= configMAX_SYSCALL_INTERRUPT_PRIORITY.
//      configASSERT is enabled, so getting this wrong is a hard hang on the
//      first tick rather than silent kernel corruption — loud, but you still
//      need to know to look here.
//   3. Cyclic tasks run their body FIRST, then vTaskDelayUntil against a
//      baseline captured before the loop. Delaying until an absolute baseline
//      is what holds a 1 kHz task at 1 kHz regardless of how long a scan took;
//      a plain vTaskDelay lets the period drift by the execution time.
//
//  And one ordering invariant: every module's init() runs single-threaded,
//  before any task is created. control's init() seeds the axis limits and
//  gains; if a COMMS scan ran first it would push a zero current limit, which
//  updateSetpoint() skips, leaving the axis on the SimpleFOC default for a scan.
// ============================================================================
#include "boot.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "config/motor_config.h"
#include "config/tasks_config.h"
#include "io/io_brake.h"
#include "io/io_can.h"
#include "io/io_gate.h"
#include "io/io_motor.h"
#include "io/io_vbus.h"
#include "app.h"
#include "state.h"

namespace boot {
namespace {

// ---------------------------------------------------------------------------
//  Fatal boot failure. Reached only before the scheduler exists, so a busy loop
//  is the right answer: there is nothing left to schedule and the power stage
//  was never armed.
// ---------------------------------------------------------------------------
void fatal(const char *msg) {
  Serial.print("[-] ");
  Serial.println(msg);
  Serial.flush();
  for (;;) {}
}

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
  // BEFORE the safety task starts reading them.
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

// ---------------------------------------------------------------------------
//  The FOC timer. The ISR must reach its task through a plain pointer read: a
//  capturing lambda would go through std::function, which heap-allocates and
//  adds an indirection inside a 20 kHz interrupt.
// ---------------------------------------------------------------------------
TaskHandle_t s_foc_task = nullptr;

void focTimerIsr() {
  BaseType_t hpw = pdFALSE;
  if (s_foc_task) vTaskNotifyGiveFromISR(s_foc_task, &hpw);
  portYIELD_FROM_ISR(hpw);
}

// ---------------------------------------------------------------------------
//  The four task bodies.
// ---------------------------------------------------------------------------

// Event-paced at FOC_TICK_HZ. Not cyclic: a 1 ms RTOS tick is useless at 20 kHz.
void focTask(void *) {
  // (1) Publish the handle BEFORE the timer can fire. See the file header.
  s_foc_task = xTaskGetCurrentTaskHandle();

  HardwareTimer *tim = new HardwareTimer(TIM6);
  tim->setOverflow(FOC_TICK_HZ, HERTZ_FORMAT);
  tim->attachInterrupt(focTimerIsr);
  tim->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);   // (2) see the file header
  tim->resume();

  for (;;) {
    // pdTRUE CLEARS the count, so a tick missed while this task was preempted
    // is lost rather than made up. That is correct for a control loop — one
    // late execution beats two back-to-back ones with a bogus dt — and it is
    // also why anything that steals CPU from this task at a fixed rate shows up
    // as a periodic disturbance rather than as jitter.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    app::foc::update();
  }
}

void safetyTask(void *) {
  TickType_t last = xTaskGetTickCount();          // (3) baseline before the loop
  for (;;) {
    app::safety::update();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SCAN_MS_SAFETY));
  }
}

// Read the fieldbus, run the logic, publish. That order is the contract: a CAN
// setpoint arriving in a given millisecond is acted on and reported in the same
// millisecond, rather than one scan later.
void commsTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    app::comms::readFieldbus();
    app::control::update();
    app::comms::publishTelemetry();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SCAN_MS_COMMS));
  }
}

void consoleTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    app::console::update();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SCAN_MS_CONSOLE));
  }
}

void createTasks() {
  struct Spec {
    const char    *name;        // short: it shows up in RTOS traces
    TaskFunction_t body;
    UBaseType_t    priority;
    uint16_t       stack_words; // depth in WORDS (4 bytes), as xTaskCreate wants
  };
  const Spec TASKS[] = {
    {"SAFE",  safetyTask,  PRIO_SAFETY,    STACK_SAFETY},
    {"FOC",   focTask,     PRIO_FOC,       STACK_FOC},
    {"COMMS", commsTask,   PRIO_COMMS,     STACK_COMMS},
    {"SER",   consoleTask, PRIO_TELEMETRY, STACK_TELEMETRY},
  };

  for (const Spec &t : TASKS) {
    if (xTaskCreate(t.body, t.name, t.stack_words, nullptr, t.priority, nullptr) != pdPASS)
      fatal("xTaskCreate FAILED (Check FreeRTOS heap)");
  }
}

}  // namespace

// ---------------------------------------------------------------------------
void run() {
  bringUpHardware();

  // Cold-start, while still single-threaded — see the file header for why this
  // must precede createTasks(). Only the modules that need one have an init();
  // the rest start from their static initialisers.
  app::control::init();   // seeds the axis limits and gains from motor_config.h

  createTasks();

  Serial.println("SAFE state (disarmed). Send 'A' via serial or CAN CLOSED_LOOP state to arm.");
  app::console::printBanner();

  vTaskStartScheduler();
  for (;;) {}   // unreachable: the scheduler does not return
}

}  // namespace boot
