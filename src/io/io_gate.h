// ============================================================================
//  io_gate.h — I/O module for the DRV8301 gate driver: its enable line, its
//  fault line and its configuration SPI.
//
//  EN_GATE is the master switch of the whole power stage. Note that it also
//  gates GVDD, the DRV8301's internal gate-regulator rail, which the AUX brake
//  driver is wired to (see io_brake.h): dropping EN_GATE kills the brake too.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "drv8301.h"
#include "config/hw_pinout.h"   // the inline accessors below name the pins

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
