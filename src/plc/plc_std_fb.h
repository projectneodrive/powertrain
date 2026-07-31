// ============================================================================
//  plc_std_fb.h — the standard function block library (IEC 61131-3, §2.5.2.3).
//
//  Header-only and stateless-until-instantiated: an unused block generates no
//  code at all, so this file costs nothing to keep complete.
//
//  CALLING CONVENTION for every FB in this codebase: operator() takes the
//  VAR_INPUTs, updates the internal state for one scan, and returns the primary
//  VAR_OUTPUT. Secondary outputs are readable as public members afterwards.
//  That keeps a POU body reading like an ST network:
//
//      if (fbOvDelay(vbus > CFG_VBUS_OV_TRIP)) { ...trip... }
//
//  TIMEBASE: the timer blocks here count SCANS, not milliseconds. Every task in
//  this configuration has a fixed period, so scans are an exact and cheaper
//  timebase — no millis() call in a 20 kHz loop, and no 49-day rollover to
//  reason about. Declare presets with scansFor() so the source still reads in
//  real time units.
// ============================================================================
#pragma once
#include <Arduino.h>

namespace plc {

// Preset helper: how many scans of a `scan_ms`-period task cover `ms`.
// Rounds up and never returns 0, so a preset shorter than one scan still means
// "the next scan", not "immediately".
constexpr uint32_t scansFor(uint32_t ms, uint32_t scan_ms) {
  return (ms + scan_ms - 1) / scan_ms ? (ms + scan_ms - 1) / scan_ms : 1;
}

// ---- LIMIT (function, §2.5.1.5.2) ------------------------------------------
constexpr float LIMIT(float mn, float in, float mx) {
  return in < mn ? mn : (in > mx ? mx : in);
}

// ---- TON — on-delay timer --------------------------------------------------
//  Q goes true once IN has been true for PT consecutive scans, and stays true
//  while IN stays true. Any false scan resets the elapsed count to zero.
//  Q is asserted ON the PT-th true scan (count >= PT), which is the convention
//  the debounces in this firmware were written with.
class TON {
 public:
  explicit TON(uint32_t pt_scans) : _pt(pt_scans) {}

  bool operator()(bool in) {
    if (!in) { _et = 0; return false; }
    if (_et < 0xFFFFFFFFu) _et++;
    return _et >= _pt;
  }

  uint32_t ET() const { return _et; }   // elapsed scans
  void     reset()    { _et = 0; }

 private:
  const uint32_t _pt;
  uint32_t       _et = 0;
};

// ---- R_TRIG / F_TRIG — edge detection --------------------------------------
class R_TRIG {
 public:
  bool operator()(bool clk) { bool q = clk && !_prev; _prev = clk; return q; }
 private:
  bool _prev = false;
};

class F_TRIG {
 public:
  bool operator()(bool clk) { bool q = !clk && _prev; _prev = clk; return q; }
 private:
  bool _prev = false;
};

// ---- RS — reset-dominant bistable ------------------------------------------
//  Reset wins when both inputs are true. This is the right default for a fault
//  latch only if a clear request may not race a live fault condition; where the
//  fault must win, evaluate set after reset in the POU body instead.
class RS {
 public:
  bool operator()(bool set, bool reset) {
    if (set)   _q = true;
    if (reset) _q = false;
    return _q;
  }
  bool Q() const { return _q; }
 private:
  bool _q = false;
};

// ---- RAMP_REAL — slew-rate limiter -----------------------------------------
//  Moves the output toward `target` by at most `max_step` per scan. A max_step
//  of 0 or less is a pass-through (direct step), matching "0 = no ramp" in the
//  configuration.
class RAMP_REAL {
 public:
  float operator()(float target, float max_step) {
    if (max_step <= 0.0f) { _out = target; return _out; }
    _out += LIMIT(-max_step, target - _out, max_step);
    return _out;
  }
  float OUT() const { return _out; }
  void  preset(float v) { _out = v; }
 private:
  float _out = 0.0f;
};

// ---- HYSTERESIS — two-threshold latch --------------------------------------
//  Q rises when IN goes strictly ABOVE `high`, and falls when IN goes strictly
//  BELOW `low`. Between the two thresholds Q holds its previous value.
//  The strict comparisons are deliberate: they are the ones the brake chopper
//  was commissioned with, and at the thresholds themselves "hold" is the safe
//  answer for both directions.
class HYSTERESIS {
 public:
  bool operator()(float in, float low, float high) {
    if (_q) { if (in < low)  _q = false; }
    else    { if (in > high) _q = true;  }
    return _q;
  }
  bool Q() const { return _q; }
  void reset()   { _q = false; }
 private:
  bool _q = false;
};

} // namespace plc
