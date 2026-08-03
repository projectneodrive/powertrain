// ============================================================================
//  pot_input.h — the spring-return potentiometer, read as a velocity joystick.
//
//  Deliberately knows nothing about CAN: poll() returns a velocity when one
//  should be sent, and the caller decides what to do with it. That is what
//  keeps this testable, and what lets the same input drive something else.
//
//  The pot's rest position is NOT its electrical mid-point, so the two sides of
//  travel are scaled independently — see potToVelocity() in the .cpp.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_config.h"

namespace pot {

class Joystick {
 public:
  void begin();

  // Sample at POT_POLL_MS. Returns true, with `vel_rad_s` filled in, only when
  // the reading moved enough to be worth a CAN frame.
  bool poll(uint32_t now_ms, float& vel_rad_s);

  // Capture the current reading as the rest point (serial 'Z'). Returns the
  // previous value so the caller can acknowledge old -> new.
  int calibrateRest();

  int rest() const { return _adc_rest; }
  int raw()  const { return _adc_last; }

 private:
  int      _adc_rest    = POT_ADC_REST_DEFAULT;   // refined at runtime by 'Z'
  int      _adc_last    = 0;
  uint32_t _last_poll   = 0;
  bool     _initialised = false;   // first sample only seeds _adc_last
};

}  // namespace pot
