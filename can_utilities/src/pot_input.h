// ============================================================================
//  pot_input.h — the spring-return potentiometer, read as a velocity joystick.
//
//  Deliberately knows nothing about CAN: poll() returns a velocity when one
//  should be sent, and the caller decides what to do with it. That is what
//  keeps this testable, and what lets the same input drive something else.
//
//  The pot's rest position is NOT its electrical mid-point, so the two sides of
//  travel are scaled independently — see potToVelocity() in the .cpp. Where
//  that rest point comes from is the other half of the story: it is MEASURED at
//  startup, because a spring-return pot is at rest when the board powers up.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_config.h"

namespace pot {

// The outcome of a rest-point measurement, reported rather than just logged so
// a caller can tell an accepted calibration from a fallback.
struct RestMeasurement {
  int  adc     = 0;      // averaged reading
  int  spread  = 0;      // max-min across the samples — the "is it moving" tell
  bool stable  = false;  // spread within POT_CAL_MAX_SPREAD
  bool inRange = false;  // clear of both rails by POT_CAL_RAIL_MARGIN

  bool ok() const { return stable && inRange; }
};

class Joystick {
 public:
  // Measure the pot and adopt the reading as the rest point.
  //
  // This replaces a startup default derived from an ohm-meter reading, which
  // assumed the measured resistance span maps linearly onto the full ADC swing.
  // It does not, and being wrong there means the joystick commands a velocity
  // while the pot is sitting physically at rest — which is exactly the failure
  // it is worth spending 64 ms of boot time to avoid.
  //
  // Retries while the reading is unstable (a hand still on the pot at
  // power-up). If every attempt is rejected it falls back to
  // POT_ADC_REST_DEFAULT and says so, in a warning naming the reason: the
  // joystick still works, just on an estimate, and 'Z' fixes it.
  //
  // Blocking — it is called from setup(), where that is the point.
  RestMeasurement begin();

  // Sample at POT_POLL_MS. Returns true, with `vel_rad_s` filled in, only when
  // the reading moved enough to be worth a CAN frame.
  bool poll(uint32_t now_ms, float& vel_rad_s);

  // Re-measure at runtime (serial 'Z'). Uses the same averaging as begin(), but
  // ACCEPTS the result even when it is rejected-looking: the operator has
  // explicitly said "this is rest", and refusing them would leave a miscalibrated
  // pot with no way out. The measurement is still reported so they can see why
  // it looked wrong.
  RestMeasurement calibrateRest();

  int  rest() const       { return _adc_rest; }
  int  raw() const        { return _adc_last; }
  // False when begin() fell back to the estimate — the joystick is usable but
  // its zero is a guess.
  bool calibrated() const { return _calibrated; }

 private:
  RestMeasurement measure() const;

  int      _adc_rest    = POT_ADC_REST_DEFAULT;
  int      _adc_last    = 0;
  uint32_t _last_poll   = 0;
  bool     _initialised = false;   // first sample only seeds _adc_last
  bool     _calibrated  = false;   // rest point was measured, not assumed
};

}  // namespace pot
