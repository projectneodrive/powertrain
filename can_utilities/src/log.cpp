// ============================================================================
//  log.cpp — see log.h.
// ============================================================================
#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace logx {
namespace {

Level g_level = (Level)LOG_DEFAULT_LEVEL;

// ---- duplicate folding -----------------------------------------------------
// Only the immediately preceding line is remembered. A ring of recent messages
// would catch alternating pairs too, but the failure this exists for is a
// message repeating in a loop, and one slot catches that for a fraction of the
// RAM and none of the search cost.
char     g_last[LOG_DEDUP_TEXT_LEN] = {0};
uint16_t g_repeats  = 0;
uint32_t g_last_ms  = 0;

// ---- rate cap --------------------------------------------------------------
// A plain token bucket over a one-second window. This is the backstop: folding
// handles a message repeating verbatim, this handles a burst of DIFFERENT
// messages, which is what a genuine fault storm looks like.
uint32_t g_window_ms = 0;
uint16_t g_in_window = 0;
uint16_t g_dropped   = 0;

char levelChar(Level lv) {
  switch (lv) {
    case LVL_ERROR: return 'E';
    case LVL_WARN:  return 'W';
    case LVL_INFO:  return 'I';
    default:        return 'D';
  }
}

// The one function that actually reaches the serial port.
//
// The timestamp is the station's millis(), matching the `t=` on the telemetry
// line so the two streams can be lined up. It is here because for a fault that
// RECURS, the period is the diagnosis — a link dropping every 3.5 s is a board
// rebooting, every 60 s is something thermal — and without a timestamp the only
// way to measure it is to sit and watch with a stopwatch.
void emit(Level lv, const char* tag, const char* text) {
  Serial.print("log ");
  Serial.print(millis());
  Serial.print(' ');
  Serial.print(levelChar(lv));
  Serial.print(' ');
  Serial.print(tag);
  Serial.print(' ');
  Serial.println(text);
}

// Report and clear a pending fold. Emitted at the level of the folded message
// would need us to remember it; SYS/INFO is enough, and keeps the summary out
// of an errors-only view where it would be the only line with no error.
void flushRepeats() {
  if (g_repeats == 0) return;
  char buf[64];
  snprintf(buf, sizeof(buf), "previous line repeated %u times", (unsigned)g_repeats);
  g_repeats = 0;
  emit(LVL_INFO, "SYS", buf);
}

void flushDropped() {
  if (g_dropped == 0) return;
  char buf[80];
  snprintf(buf, sizeof(buf), "%u lines dropped (log rate cap, %u/s) - raise it or lower the level",
           (unsigned)g_dropped, (unsigned)LOG_MAX_LINES_PER_S);
  g_dropped = 0;
  emit(LVL_WARN, "SYS", buf);
}

}  // namespace

void setLevel(Level lv) {
  g_level = (lv < LVL_COUNT) ? lv : LVL_DEBUG;
}

Level level() { return g_level; }

const char* levelName(Level lv) {
  switch (lv) {
    case LVL_ERROR: return "ERROR";
    case LVL_WARN:  return "WARN";
    case LVL_INFO:  return "INFO";
    case LVL_DEBUG: return "DEBUG";
    default:        return "?";
  }
}

void msg(Level lv, const char* tag, const char* fmt, ...) {
  if (lv > g_level) return;

  char text[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);

  const uint32_t now = millis();

  // Fold a verbatim repeat. Note this happens BEFORE the rate cap: a repeating
  // message must not spend the budget that a different, newer message needs.
  if (g_repeats < 0xFFFF && strncmp(text, g_last, sizeof(g_last)) == 0 &&
      (now - g_last_ms) < LOG_DEDUP_MS) {
    ++g_repeats;
    g_last_ms = now;
    return;
  }

  // Token bucket. Errors are exempt: the whole point of the cap is to keep the
  // log readable, and dropping the one line that says what broke would defeat
  // it. An error storm is bounded by the folding above.
  if (now - g_window_ms >= 1000) {
    g_window_ms = now;
    g_in_window = 0;
    flushDropped();
  }
  if (lv != LVL_ERROR && g_in_window >= LOG_MAX_LINES_PER_S) {
    ++g_dropped;
    return;
  }
  ++g_in_window;

  flushRepeats();
  emit(lv, tag, text);

  strncpy(g_last, text, sizeof(g_last) - 1);
  g_last[sizeof(g_last) - 1] = '\0';
  g_last_ms = now;
}

void tick(uint32_t now_ms) {
  // A message that repeated and then stopped would otherwise sit on an
  // unreported count until something else happens to be logged — which, on a
  // quiet bus, can be never.
  if (g_repeats > 0 && (now_ms - g_last_ms) >= LOG_DEDUP_MS) {
    flushRepeats();
  }
  if (g_dropped > 0 && (now_ms - g_window_ms) >= 1000) {
    g_window_ms = now_ms;
    g_in_window = 0;
    flushDropped();
  }
}

}  // namespace logx

// ============================ axis vocabulary ==============================

namespace axisnames {

const char* state(uint8_t s) {
  switch (s) {
#define AXIS_STATE(name, value, label) case value: return label;
#define AXIS_MODE(name, value, label)
#define AXIS_ERROR(name, value, label)
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
    default: return "?";
  }
}

const char* mode(uint8_t m) {
  switch (m) {
#define AXIS_STATE(name, value, label)
#define AXIS_MODE(name, value, label) case value: return label;
#define AXIS_ERROR(name, value, label)
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
    default: return "?";
  }
}

void errors(uint32_t err, char* buf, size_t len) {
  if (len == 0) return;
  if (err == 0) {
    strncpy(buf, "none", len - 1);
    buf[len - 1] = '\0';
    return;
  }

  size_t used = 0;
  uint32_t named = 0;
  buf[0] = '\0';

  auto append = [&](const char* s) {
    const size_t room = (used + 1 < len) ? (len - used - 1) : 0;
    if (room == 0) return;
    strncat(buf + used, s, room);
    used += strnlen(s, room);
  };

  append("[");
  bool first = true;

#define AXIS_STATE(name, value, label)
#define AXIS_MODE(name, value, label)
#define AXIS_ERROR(name, value, label)      \
  if (err & (uint32_t)(value)) {            \
    if (!first) append("|");                \
    append(label);                          \
    first = false;                          \
    named |= (uint32_t)(value);             \
  }
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE

  // Anything this build has no name for. Dropping it would make a newer
  // firmware's error look like no error at all.
  const uint32_t unknown = err & ~named;
  if (unknown != 0) {
    char hex[16];
    snprintf(hex, sizeof(hex), "%s+0x%lX", first ? "" : "|", (unsigned long)unknown);
    append(hex);
  }
  append("]");
}

}  // namespace axisnames
