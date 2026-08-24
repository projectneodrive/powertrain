// ============================================================================
//  bridge_console.h — the serial console the host GUI drives.
//
//  Its command set is NOT defined here. It is the firmware's own
//  (../include/console_commands.h) plus the station's extras
//  (bridge_commands.h), concatenated into one dispatch table. That is the whole
//  design: a keystroke means the same thing whether the GUI is plugged into the
//  board's USB port or into this station's, and neither list can drift from the
//  other because there is only one of them.
//
//  It also means this station has to have an opinion about every command the
//  board has. Several of them — the current PID, the D terms, hall calibration
//  — have no CANSimple representation at all, so the honest answer is "not over
//  CAN, use the board's own port", printed as a normal acknowledgement. The
//  build will not let that question go unanswered: a new line in the firmware's
//  table fails this link until a handler exists.
//
//  BEFORE THIS EXISTED the two sets had already drifted: 'P' meant a position
//  setpoint here and the position-PID family on the board, so a GUI sending
//  "P1.5" to the two ports did two unrelated things.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_axis.h"
#include "bridge_state.h"
#include "pot_input.h"

namespace bridge {
namespace console {

// Everything the handlers act on. Passed once at boot rather than reached
// through globals, so the dependency is visible from the outside.
struct Context {
  Axis*          axis  = nullptr;
  State*         state = nullptr;
  pot::Joystick* pot   = nullptr;
};

// Wire up the handlers and check the two command tables for key collisions.
void begin(const Context& ctx);

// Drain the serial port and dispatch complete lines. Non-blocking.
void poll();

// The help banner, generated from the same table as the dispatch.
void printBanner();

}  // namespace console
}  // namespace bridge
