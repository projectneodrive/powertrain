// ============================================================================
//  io.h — the board's I/O layer: one namespace per piece of hardware.
//
//  These five were a header/source pair each, which made ten files to say what
//  amounts to "here is the gate driver, the brake bridge, the bus ADC, the CAN
//  controller and the console". They are small, they change together and they
//  are all the same kind of thing, so they live together.
//
//  The motor is NOT here (io_motor.h): it drags SimpleFOC and the sensor
//  selection in with it, and only the modules that actually drive the motor
//  should pay for that.
//
//  EVERYTHING IN THIS FILE TOUCHES REGISTERS OR PINS AND NOTHING ELSE. No
//  control logic, no policy, no state machine — those live in src/app/, which
//  is what lets either side be read without the other.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "drv8301.h"
#include "config/hw_pinout.h"   // the inline accessors below name the pins
#include "odrive_can.h"

// ============================================================================
//  io_gate.h — I/O module for the DRV8301 gate driver: its enable line, its
//  fault line and its configuration SPI.
//
//  EN_GATE is the master switch of the whole power stage. Note that it also
//  gates GVDD, the DRV8301's internal gate-regulator rail, which the AUX brake
//  driver is wired to (see io_brake.h): dropping EN_GATE kills the brake too.
// ============================================================================
namespace io {
namespace gate {

// The DRV8301 configuration SPI object, exposed for the boot-time status dump.
extern DRV8301 drv;

// GPIO setup + the DRV8301 hardware reset pulse (EN_GATE LOW -> HIGH). Runs
// before Serial is up, so it prints nothing.
void preInit();

// Bring the DRV8301 out of reset over SPI and program the amplifier gain.
// Prints the status register and whether the gain took (a FAIL here means SPI).
void init();

// Re-assert the amplifier gain. Needed after every motor.enable(): the DRV8301
// needs a settling delay before its SPI is reliable again.
void setGain();

inline void enable()  { digitalWrite(PIN_EN_GATE, HIGH); }
inline void disable() { digitalWrite(PIN_EN_GATE, LOW); }
inline bool enabled() { return digitalRead(PIN_EN_GATE) == HIGH; }

// nFAULT is active LOW. Raw read: the debounce lives in fb/fb_gate_fault.h.
inline bool faultAsserted() { return digitalRead(PIN_N_FAULT) == LOW; }

} // namespace gate
} // namespace io

// ============================================================================
//  io_brake.h — I/O module for the brake-resistor half-bridge (AUX terminals).
//
//  This module owns TIM2/CH3/CH4 and nothing else. TIM1 (motor PWM), TIM3
//  (sensor) and TIM6 (FOC tick) are never reallocated.
//
//  It is PURE OUTPUT: give it a duty, it programs the timer. The decision of
//  WHAT duty to apply — the hysteresis and the proportional law — lives in
//  fb/fb_brake_chopper.h, because that is control logic, not hardware.
//
//  Hardware: DRV8301/ODESC AUX half-bridge — LM5109B gate driver (U7) +
//  NTMFS5C628N (IC15//IC16 high side, IC13//IC14 low side). The resistor is
//  wired between the midpoint (JP2.2 / TP13) and ground (JP2.1 / TP12), so it
//  is the HIGH FET that dissipates. The topology and the mandatory complementary
//  drive are documented in config/hw_pinout.h.
//
//  /!\ CRITICAL HARDWARE DEPENDENCY: the LM5109B's VDD is fed from GVDD, the
//  DRV8301's internal gate regulator, which only exists while EN_GATE is high.
//  The brake is therefore PHYSICALLY INOPERATIVE with the stage disarmed — that
//  is not a software decision, it is the board's wiring. Direct consequence:
//  there is NO over-voltage protection at all while the stage is disarmed (a
//  motor driven mechanically at standstill, for instance).
//  TODO: if permanent protection is needed, hold EN_GATE high at all times and
//  cut the motor another way (TIM1's BDTR.MOE = 0 puts the six motor gates in
//  Hi-Z without touching EN_GATE, hence without losing GVDD).
// ============================================================================
namespace io {
namespace brake {

// Drive both AUX gates LOW as plain GPIO. Must run at the very top of boot,
// BEFORE anything switches those pins to an alternate function, so the
// half-bridge never passes through an undefined state at power-up.
void preInit();

// Configure TIM2 center-aligned and leave the half-bridge STOPPED.
// Call from the boot sequence AFTER the DRV8301 has been brought up (GVDD).
void init();

// Apply a duty in [0 .. CFG_BRAKE_MAX_DUTY]. Values <= 0 stop the bridge.
void setDuty(float d);

// Immediate stop of both FETs. Safe to call before init() and from any
// context (two register writes).
void off();

// Duty currently applied, for telemetry.
float duty();

} // namespace brake
} // namespace io

// ============================================================================
//  io_vbus.h — I/O module for the DC bus voltage measurement.
//
//  Owns a DEDICATED ADC, never the one the phase-current shunts use. The
//  bench history of what goes wrong otherwise is in config/hw_pinout.h next to
//  PIN_VBUS — it is not a theoretical concern, it produced a systematic false
//  over-voltage fault during alignment.
//
//  readRaw() is a BLOCKING one-shot conversion (~24 us). Exactly one program
//  may call it (PRG_SAFETY): there is no concurrency protection here, and a
//  second caller would interleave HAL_ADC_Start/Stop with the first.
// ============================================================================
namespace io {
namespace vbus {

// Claims whichever of ADC1/ADC2 the current sense did not take, configures PA6
// and prints which ADC went where. Returns false if the HAL refused the
// configuration, in which case readRaw() returns -1 forever.
bool init();

// One blocking conversion. Returns the volts AT THE PIN (before the divider),
// or -1.0f on error.
float readRaw();

} // namespace vbus
} // namespace io

// ============================================================================
//  io_can.h — I/O module for the CANSimple fieldbus.
//
//  A thin binding: it owns the single OdriveCAN instance and maps it onto the
//  process image's axis block (gvl::AXIS). The protocol itself lives in
//  lib/odrive_can; the commands it answers are declared in
//  include/can_commands.h.
// ============================================================================
namespace io {
namespace can {

extern odcan::OdriveCAN bus;

// Bring up CAN1 and print the node/bit-rate line.
void init();

} // namespace can
} // namespace io

// ============================================================================
//  io_console.h — I/O module for the USB serial console.
//
//  Two jobs, both of them plumbing: assembling incoming bytes into lines, and
//  emitting the acknowledgement format every command replies with. The command
//  set itself is declared in include/console_commands.h and executed by
//  PRG_CONSOLE.
//
//  ACK FORMAT: a synchronous reply, with no timestamp, showing the old -> new
//  value (or just acknowledging the request). The absence of a timestamp is what
//  lets the GUI tell an acknowledgement apart from a telemetry line ("t=...").
// ============================================================================
namespace io {
namespace console {

// Consumes buffered serial bytes. Returns a NUL-terminated line as soon as one
// is complete, or nullptr when the buffer runs dry mid-line. Empty lines are
// swallowed. Call it in a loop to drain everything that arrived.
// The returned pointer is valid until the next call.
const char* poll();

// ---- Acknowledgement helpers ----------------------------------------------
// "AK <tag>: <field> <old> -> <new> <unit>"
void ackFloat(const char* tag, const char* field, float oldv, float newv,
              uint8_t prec, const char* unit = nullptr);
// "AK <tag>: <field> <old> -> <new>"   (integers / booleans)
void ackInt(const char* tag, const char* field, long oldv, long newv);
// "AK <tag>: <message>"
void ackMsg(const char* tag, const char* message);

} // namespace console
} // namespace io
