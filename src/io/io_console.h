// ============================================================================
//  io_console.h — I/O module for the USB serial console.
//
//  Two jobs, both of them plumbing: assembling incoming bytes into lines, and
//  emitting the acknowledgement format every command replies with. The command
//  set itself is declared in include/console_commands.h and executed by
//  PRG_CONSOLE.
//
//  ACK FORMAT: a synchronous reply, with no timestamp, showing the old -> new
//  value (or just acknowledging the request). The absence of a timestamp is what
//  lets the GUI tell an acknowledgement apart from a telemetry line ("t=...").
// ============================================================================
#pragma once
#include <Arduino.h>

namespace io {
namespace console {

// Consumes buffered serial bytes. Returns a NUL-terminated line as soon as one
// is complete, or nullptr when the buffer runs dry mid-line. Empty lines are
// swallowed. Call it in a loop to drain everything that arrived.
// The returned pointer is valid until the next call.
const char* poll();

// ---- Acknowledgement helpers ----------------------------------------------
// "AK <tag>: <field> <old> -> <new> <unit>"
void ackFloat(const char* tag, const char* field, float oldv, float newv,
              uint8_t prec, const char* unit = nullptr);
// "AK <tag>: <field> <old> -> <new>"   (integers / booleans)
void ackInt(const char* tag, const char* field, long oldv, long newv);
// "AK <tag>: <message>"
void ackMsg(const char* tag, const char* message);

} // namespace console
} // namespace io
