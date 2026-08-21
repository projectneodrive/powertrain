// ============================================================================
//  pot_input.cpp — see pot_input.h.
// ============================================================================
#include "pot_input.h"

#include "log.h"

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

// Average POT_CAL_SAMPLES readings, and report the spread while we are at it.
// The spread is what actually decides whether the number means anything: a pot
// held still reads within a few counts, one being moved reads hundreds apart,
// and a pin with nothing connected wanders more than either.
RestMeasurement Joystick::measure() const {
  RestMeasurement m;

  long sum = 0;
  int lo = POT_ADC_MAX;
  int hi = POT_ADC_MIN;
  for (int i = 0; i < POT_CAL_SAMPLES; i++) {
    const int v = analogRead(POT_PIN);
    sum += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    delay(POT_CAL_SAMPLE_MS);
  }

  m.adc     = (int)(sum / POT_CAL_SAMPLES);
  m.spread  = hi - lo;
  m.stable  = m.spread <= POT_CAL_MAX_SPREAD;
  m.inRange = (m.adc >= POT_ADC_MIN + POT_CAL_RAIL_MARGIN) &&
              (m.adc <= POT_ADC_MAX - POT_CAL_RAIL_MARGIN);
  return m;
}

RestMeasurement Joystick::begin() {
  RestMeasurement m;

  for (int attempt = 1; attempt <= POT_CAL_ATTEMPTS; attempt++) {
    m = measure();
    if (m.ok()) {
      _adc_rest   = m.adc;
      _calibrated = true;
      LOG_I("POT", "rest calibrated at boot: adc=%d (spread %d, estimate was %d)",
            m.adc, m.spread, (int)POT_ADC_REST_DEFAULT);
      break;
    }
    LOG_W("POT", "rest calibration attempt %d/%d rejected: adc=%d spread=%d (%s)",
          attempt, (int)POT_CAL_ATTEMPTS, m.adc, m.spread,
          !m.stable ? "reading unstable - pot being moved, or nothing connected to the pin"
                    : "sitting on a rail - full deflection, or a broken wiper");
  }

  if (!_calibrated) {
    // Usable, but its zero is the ohm-meter estimate rather than a measurement.
    // Said once, loudly, because the symptom otherwise looks like a possessed
    // motor: a small commanded velocity with the pot physically at rest.
    _adc_rest = POT_ADC_REST_DEFAULT;
    LOG_W("POT", "using the estimated rest point %d - hold the pot at rest and send Z",
          (int)POT_ADC_REST_DEFAULT);
  }

  // Seeding this from the same measurement is what stops the very first poll()
  // from looking like a large operator movement and firing a command.
  _adc_last    = m.adc;
  _initialised = false;
  _last_poll   = 0;
  return m;
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

RestMeasurement Joystick::calibrateRest() {
  const RestMeasurement m = measure();

  // Accepted either way — see the header. A warning is enough: the operator
  // asked for this, and the numbers below tell them if it looked wrong.
  if (!m.stable) {
    LOG_W("POT", "rest set to %d but the reading was unstable (spread %d) - "
                 "hold the pot still and send Z again", m.adc, m.spread);
  } else if (!m.inRange) {
    LOG_W("POT", "rest set to %d, which is within %d counts of a rail - one "
                 "direction will have almost no travel", m.adc, (int)POT_CAL_RAIL_MARGIN);
  }

  _adc_rest   = m.adc;
  _calibrated = true;
  // The pot has not moved, but the reference it is measured against just did:
  // without this the next poll() reads the same jump as an operator input.
  _adc_last   = m.adc;
  return m;
}

}  // namespace pot
