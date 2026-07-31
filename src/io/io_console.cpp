// ============================================================================
//  io_console.cpp — see io_console.h.
// ============================================================================
#include "io/io_console.h"

namespace io {
namespace console {
namespace {
char    g_buf[24];
uint8_t g_idx = 0;
}

const char* poll() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      uint8_t len = g_idx;
      g_buf[len] = '\0';
      g_idx = 0;
      if (len > 0) return g_buf;   // empty lines are swallowed, keep draining
    } else if (g_idx < sizeof(g_buf) - 1) {
      g_buf[g_idx++] = c;
    }
  }
  return nullptr;
}

void ackFloat(const char* tag, const char* field, float oldv, float newv,
              uint8_t prec, const char* unit) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.print(field); Serial.print(' ');
  Serial.print(oldv, prec);
  Serial.print(" -> ");
  Serial.print(newv, prec);
  if (unit) { Serial.print(' '); Serial.print(unit); }
  Serial.println();
}

void ackInt(const char* tag, const char* field, long oldv, long newv) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.print(field); Serial.print(' ');
  Serial.print(oldv);
  Serial.print(" -> ");
  Serial.println(newv);
}

void ackMsg(const char* tag, const char* message) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.println(message);
}

} // namespace console
} // namespace io
