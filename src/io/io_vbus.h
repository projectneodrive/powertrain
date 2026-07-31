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
#pragma once
#include <Arduino.h>

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
