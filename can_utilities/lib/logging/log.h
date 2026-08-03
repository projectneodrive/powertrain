// ============================================================================
//  log.h — the control station's event log.
//
//  THE PROBLEM THIS SOLVES. The station used to print a line per transmitted
//  frame, a nine-field bus-counter dump every 2 s, and a joystick line at 10 Hz,
//  all interleaved with the 10 Hz telemetry. The web GUI's monitor pane scrolled
//  faster than anyone could read, so the lines that actually mattered — a new
//  error bit, a lost link, a rejected command — went past unseen. A log nobody
//  can read is worse than no log: it looks like diagnostics.
//
//  THE RULES HERE
//
//   1. LEVELS. Every line has one, and only lines at or above the current level
//      are emitted. The default is INFO, so the frame trace (DEBUG) is off until
//      asked for. Set it at runtime with the console's `D<n>`.
//
//   2. EDGES, NOT STATES. Anything that is *true for a while* — a bus alert, an
//      axis state, an error word — is reported with OnChange<> below, so it
//      prints when it BECOMES true and not on every scan that observes it. This
//      is what turns a screenful into one line.
//
//   3. PERIODIC DATA IS NOT LOGGING. Counters and status belong on the machine
//      -readable `can ...` line (see can_diag.h), which the GUI's CAN Devices
//      page reads and which never reaches the monitor pane at all.
//
//   4. NOTHING GETS THROUGH UNCAPPED. Identical consecutive messages are folded
//      into a "repeated N times" summary, and a hard token bucket limits the
//      whole stream. Even a bug that logs in a tight loop cannot drown the GUI.
//
//  WIRE FORMAT — one line, parsed by the GUI (src/logevent.h):
//
//      log <sev> <tag> <free text>
//
//    <sev>  one of E W I D
//    <tag>  short uppercase category: SYS CAN BUS LINK AXIS POT
//
//  Lines that do not match are still shown, as INFO — that is what keeps the
//  firmware's own banner text and `AK ...` acknowledgements working unchanged.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_config.h"

namespace logx {

enum Level : uint8_t {
  LVL_ERROR = 0,   // something is broken and will stay broken
  LVL_WARN  = 1,   // degraded, or about to be
  LVL_INFO  = 2,   // state changes worth seeing — the default ceiling
  LVL_DEBUG = 3,   // per-frame trace; deliberately noisy
  LVL_COUNT = 4,
};

void  setLevel(Level lv);
Level level();
const char* levelName(Level lv);

// Format and emit one line. Prefer the macros: they skip the formatting cost
// entirely when the level is filtered out.
void msg(Level lv, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

// Emit any pending "repeated N times" / "N lines dropped" summary once its
// window has closed. Call once per scan; cheap, and without it a message that
// repeats and then stops would leave its final count unreported.
void tick(uint32_t now_ms);

#define LOG_AT(lv, tag, ...)                                  \
  do {                                                        \
    if ((lv) <= ::logx::level()) ::logx::msg(lv, tag, __VA_ARGS__); \
  } while (0)

#define LOG_E(tag, ...) LOG_AT(::logx::LVL_ERROR, tag, __VA_ARGS__)
#define LOG_W(tag, ...) LOG_AT(::logx::LVL_WARN,  tag, __VA_ARGS__)
#define LOG_I(tag, ...) LOG_AT(::logx::LVL_INFO,  tag, __VA_ARGS__)
#define LOG_D(tag, ...) LOG_AT(::logx::LVL_DEBUG, tag, __VA_ARGS__)

// ---------------------------------------------------------------------------
//  OnChange — the edge detector rule 2 is built on.
//
//    if (_state_changed(hb_state)) LOG_I("AXIS", "state %s -> %s", ...);
//
//  Returns true only when the value differs from the one it last returned true
//  for, so the *first* observation also fires: "it started out faulted" is
//  exactly as interesting as "it just became faulted".
//
//  Check first() before phrasing that edge as a transition. On the first
//  observation there is no previous value — previous() returns the current one,
//  and a caller that ignores this prints "state IDLE -> IDLE".
// ---------------------------------------------------------------------------
template <typename T>
class OnChange {
 public:
  bool operator()(T value) {
    const bool had_value = _seen;
    if (_seen && value == _last) return false;
    _prev  = _seen ? _last : value;
    _last  = value;
    _seen  = true;
    _first = !had_value;
    return true;
  }
  T    previous() const { return _prev; }   // only meaningful when !first()
  T    current()  const { return _last; }
  bool first()    const { return _first; }  // this edge was the first observation
  bool seen()     const { return _seen; }
  void reset()          { _seen = false; }

 private:
  T    _last{};
  T    _prev{};
  bool _seen  = false;
  bool _first = false;
};

}  // namespace logx
