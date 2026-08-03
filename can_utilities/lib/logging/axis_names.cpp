// ============================================================================
//  axis_names.cpp — see axis_names.h.
// ============================================================================
#include "axis_names.h"

#include <stdio.h>
#include <string.h>

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
