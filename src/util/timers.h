// ============================================================================
//  util/timers.h — the small stateful helpers the control modules are built on.
//
//  Three of them, and each has real call sites. (The previous library also
//  carried R_TRIG, F_TRIG, RS and RAMP_REAL, none of which was ever used —
//  RAMP_REAL was written and then explicitly declined by the velocity ramp.)
//
//  CALLING CONVENTION: operator() takes the inputs, advances the state by one
//  call, and returns the primary output. Secondary outputs are public members
//  readable afterwards. That keeps a control body reading as a sequence of
//  decisions:
//
//      if (_ov_trip(vbus > CFG_VBUS_OV_TRIP)) { ...trip... }
//
//  TIMEBASE. Debounce counts CALLS, not milliseconds: every task here has a
//  fixed period, so calls are an exact and cheaper timebase — no millis() in a
//  20 kHz loop and no 49-day rollover to reason about.
//
//  The catch, and why Debounce takes two arguments: a preset in calls is only
//  meaningful next to the period of whatever calls it, and those two used to
//  live in different files. The over-voltage debounce computed its preset for a
//  200 Hz caller while being instantiated in a program that runs at 1 kHz and
//  is gated down by a divider elsewhere — correct, but only by coincidence of
//  two numbers nobody could see together. So the preset is declared in
//  MILLISECONDS and converted here, with the call period passed in beside it:
//
//      Debounce _gate_fault{11, SCAN_MS_SAFETY};        // 11 ms at 1 kHz
//      Debounce _ov_trip{10, 1000 / CFG_BUS_SAFETY_HZ}; // 10 ms at 200 Hz
//
//  Get the period wrong and the duration is wrong, exactly as before — but now
//  the mistake is visible at the declaration instead of implied by a divider in
//  another translation unit.
// ============================================================================
#pragma once
#include <Arduino.h>

namespace util {

// How many calls of a `period_ms` task cover `ms`. Rounds up and never returns
// 0, so a preset shorter than one period still means "the next call", not
// "immediately".
constexpr uint32_t callsFor(uint32_t ms, uint32_t period_ms) {
  return (ms + period_ms - 1) / period_ms ? (ms + period_ms - 1) / period_ms : 1;
}

// Clamp. Free function: it has no state, so making it a class would imply one.
constexpr float limit(float mn, float in, float mx) {
  return in < mn ? mn : (in > mx ? mx : in);
}

// ---- Debounce — on-delay -----------------------------------------------------
//  Output goes true once the input has been true for `ms` worth of consecutive
//  calls, and stays true while the input stays true. Any false call resets the
//  elapsed count. The output asserts ON the Nth true call (count >= preset),
//  which is the convention the debounces in this firmware were commissioned
//  with — do not "fix" it to > without re-measuring them.
class Debounce {
 public:
  // `ms` is the delay; `period_ms` is how often YOU will call this.
  constexpr Debounce(uint32_t ms, uint32_t period_ms)
      : _preset(callsFor(ms, period_ms)) {}

  bool operator()(bool in) {
    if (!in) { _elapsed = 0; return false; }
    if (_elapsed < 0xFFFFFFFFu) _elapsed++;
    return _elapsed >= _preset;
  }

  uint32_t elapsed() const { return _elapsed; }   // in calls
  void     reset()         { _elapsed = 0; }

 private:
  const uint32_t _preset;
  uint32_t       _elapsed = 0;
};

// ---- Hysteresis — two-threshold latch ----------------------------------------
//  Rises when the input goes strictly ABOVE `high`, falls when it goes strictly
//  BELOW `low`, and holds between them. The strict comparisons are deliberate:
//  they are the ones the brake chopper was commissioned with, and at the
//  thresholds themselves "hold" is the safe answer in both directions.
class Hysteresis {
 public:
  bool operator()(float in, float low, float high) {
    if (_q) { if (in < low)  _q = false; }
    else    { if (in > high) _q = true;  }
    return _q;
  }
  bool output() const { return _q; }
  void reset()       { _q = false; }

 private:
  bool _q = false;
};

}  // namespace util
