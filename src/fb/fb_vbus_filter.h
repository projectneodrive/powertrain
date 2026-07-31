// ============================================================================
//  fb_vbus_filter.h — FUNCTION_BLOCK VbusFilter
//
//  Conditions one raw ADC sample of the DC bus into the filtered voltage every
//  other block reads.
//
//    VAR_INPUT   raw_pin_v : volts at the pin, or < 0 for "conversion failed"
//    VAR_OUTPUT  (return)  : filtered bus volts, or <= 0 until primed
//
//  Median-of-3 then a first-order low-pass, in that order. The median is not
//  cosmetic: it rejects a single wild sample — a conversion aborted by an
//  injected trigger, a commutation transient — BEFORE it can enter the filter
//  state, where a low-pass would smear it across the next several samples and
//  drag the average toward a value that was never real.
//
//  An invalid sample holds the output rather than resetting it: losing one
//  conversion should not make the bus voltage momentarily unknown.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "config/motor_config.h"

namespace fb {

class VbusFilter {
 public:
  float operator()(float raw_pin_v) {
    if (raw_pin_v >= 0.0f) {
      float v = raw_pin_v * CFG_VBUS_DIV;
      if (_s0 < 0.0f) { _s0 = v; _s1 = v; }          // prime
      float m = median3(_s0, _s1, v);
      _s0 = _s1; _s1 = v;
      // tau ~ 2 ms at CFG_BUS_SAFETY_HZ
      _filt = (_filt <= 0.0f) ? m : _filt + 0.33f * (m - _filt);
    }
    return _filt;
  }

  float OUT() const { return _filt; }

 private:
  static float median3(float a, float b, float c) {
    return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
  }

  float _s0   = -1.0f;   // the two previous RAW samples
  float _s1   = -1.0f;
  float _filt = 0.0f;
};

} // namespace fb
