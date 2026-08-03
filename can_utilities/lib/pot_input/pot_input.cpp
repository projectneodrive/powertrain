// ============================================================================
//  pot_input.cpp — see pot_input.h.
// ============================================================================
#include "pot_input.h"

namespace pot {
namespace {

// Maps a raw ADC reading to a velocity command, treating the calibrated rest
// point as a (non-centered) neutral: readings within POT_REST_DEADBAND_ADC of
// rest snap to exactly 0 rad/s, and each side of rest is independently scaled
// to its own full +/-POT_VEL_MAX_RAD_S range. The independent scaling is not a
// nicety — the low side spans far fewer ADC counts than the high side, since
// rest sits near the top of travel, so a single symmetric scale would make one
// direction unusable.
float potToVelocity(int adc, int rest) {
  const int hi_edge = rest + POT_REST_DEADBAND_ADC;
  const int lo_edge = rest - POT_REST_DEADBAND_ADC;

  if (adc >= lo_edge && adc <= hi_edge) {
    return 0.0f;
  }
  if (adc > hi_edge) {
    const float span = (float)(POT_ADC_MAX - hi_edge);
    const float frac = span > 0.0f ? (adc - hi_edge) / span : 0.0f;
    return constrain(frac, 0.0f, 1.0f) * POT_VEL_MAX_RAD_S;
  }
  const float span = (float)(lo_edge - POT_ADC_MIN);
  const float frac = span > 0.0f ? (lo_edge - adc) / span : 0.0f;
  return -constrain(frac, 0.0f, 1.0f) * POT_VEL_MAX_RAD_S;
}

}  // namespace

void Joystick::begin() {
  _adc_last    = analogRead(POT_PIN);
  _adc_rest    = POT_ADC_REST_DEFAULT;
  _initialised = false;
}

bool Joystick::poll(uint32_t now_ms, float& vel_rad_s) {
  if (now_ms - _last_poll < POT_POLL_MS) {
    return false;
  }
  _last_poll = now_ms;

  const int adc   = analogRead(POT_PIN);
  const int delta = abs(adc - _adc_last);

  // Always track the latest reading, even on ticks we do not act on, and NEVER
  // reject a large jump as a spike. A spring-return pot snapping back from full
  // deflection to rest crosses a couple of thousand counts inside one 100 ms
  // poll. If that were rejected and _adc_last left stale, every subsequent
  // (now legitimate) reading near rest would ALSO look like a huge jump from
  // that stale reference and be rejected too — permanently latching the last
  // velocity sent. That was observed: the target stuck at +/-10 rad/s and never
  // returned to 0. No spike filter is worth that on a motor velocity input;
  // potToVelocity() already clamps, so a genuinely noisy sample costs one
  // 100 ms tick at the rail, not a lockup.
  _adc_last = adc;

  if (!_initialised) {
    _initialised = true;
    return false;
  }
  if (delta < POT_CHANGE_DEADBAND) {
    return false;   // ADC noise, not an operator input
  }

  vel_rad_s = potToVelocity(adc, _adc_rest);
  return true;
}

int Joystick::calibrateRest() {
  const int old_rest = _adc_rest;
  _adc_rest = analogRead(POT_PIN);
  return old_rest;
}

}  // namespace pot
