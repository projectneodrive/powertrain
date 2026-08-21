// ============================================================================
//  can_bridge.h — the control station, assembled.
//
//  Owns the five pieces and runs the scan that connects them. main.cpp does
//  nothing but call begin() and poll(); everything a reader needs in order to
//  understand the program is the poll() body in the .cpp.
//
//      Serial (host GUI)                             CAN bus
//            |                                          |
//            v                                          v
//      bridge_console  --commands-->  bridge_axis  <--frames-->  cansimple
//            ^                             |                        |
//            |                             v                     can_diag
//      bridge_telemetry  <--readings--  bridge_state              (trace,
//            |                             ^                       alerts,
//            v                             |                      counters)
//      Serial (host GUI)              pot_input
//
//  bridge_state is the only thing the two directions share, and only the CAN
//  decoders write its measured half — the same single-writer discipline the
//  firmware's process image uses, for the same reason.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_axis.h"
#include "bridge_console.h"
#include "bridge_state.h"
#include "bridge_telemetry.h"
#include "can_diag.h"
#include "cansimple.h"
#include "pot_input.h"

namespace bridge {

class ControlStation {
 public:
  // Bring up serial and the bus, then put the axis in a known configuration.
  void begin();

  // One scan. Non-blocking; call it as fast as the loop will go. The RX queue
  // is drained first, so a command issued this scan acts on this scan's
  // readings — the same read-inputs / run-logic / write-outputs order the
  // board's own COMMS task uses.
  void poll();

 private:
  cansimple::Link      _link{BRIDGE_TARGET_NODE_ID};
  candiag::Diagnostics _diag{BRIDGE_TARGET_NODE_ID};
  State                _state;
  Axis                 _axis{_link, _state};
  pot::Joystick        _pot;

  // Start of the previous scan, for the stall check at the top of poll().
  uint32_t             _last_scan_ms = 0;
};

extern ControlStation station;

}  // namespace bridge
