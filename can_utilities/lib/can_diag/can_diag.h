// ============================================================================
//  can_diag.h — bus diagnostics for the control station.
//
//  Everything here is pure observation: nothing influences what is transmitted.
//  It answers two different questions with two different mechanisms, and the
//  split is the reason the monitor pane is readable.
//
//    EVENTS  -> lib/logging. Something CHANGED: a node appeared, the controller
//               went bus-off, error counters started climbing. Edge-triggered,
//               deduplicated, rate-capped.
//
//    STATE   -> readBus(), fed into the machine-readable `can ...` line by
//               bridge_telemetry.cpp. Counters and controller state, sampled at
//               1 Hz for the GUI's CAN Devices page. Never printed as prose.
//
//  It used to print the second kind as prose every 2 s — nine fields of numbers
//  that are only meaningful as a trend, in a pane where you can see about four
//  seconds of history. That is now a table on a page instead.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "cansimple.h"
#include "log.h"

namespace candiag {

// A sample of the TWAI controller's own state. Field names match the `can ...`
// line's keys so the mapping stays greppable from either end.
struct BusStatus {
  int      state       = 0;   // twai_state_t: 0 stopped, 1 running, 2 bus-off, 3 recovering
  uint32_t tx_ec       = 0;   // transmit error counter
  uint32_t rx_ec       = 0;   // receive error counter
  uint32_t tx_failed   = 0;
  uint32_t rx_missed   = 0;
  uint32_t rx_overrun  = 0;
  uint32_t arb_lost    = 0;
  uint32_t bus_ec      = 0;   // bus error count
};

const char* busStateName(int state);

class Diagnostics {
 public:
  explicit Diagnostics(uint8_t target_node) : _target(target_node) {}

  // The Link's frame tap is a plain function pointer, so bind() records the
  // instance it forwards to. One station drives one bus; a second is a bug.
  static void bind(Diagnostics* self) { _self = self; }
  static void tap(const char* dir, const cansimple::Frame& frame, bool ok);

  // Print the first time each node id is seen. A board whose CFG_CAN_NODE_ID
  // does not match this station announces itself in one line, instead of
  // presenting as "no telemetry" with no explanation.
  void noteNodeSeen(uint8_t node);

  // Bitmask of node ids observed, for the `can ...` line's `nodes=` field —
  // which is how the GUI lists devices it was never configured to expect.
  uint64_t seenNodes() const { return _seen_nodes; }

  // Drain the TWAI alert flags. Call every scan; does not block. Alerts that
  // fire continuously on a marginal link are folded by the logger rather than
  // filtered here, so the count is reported rather than quietly lost.
  void pollAlerts();

  // Sample the controller. Also raises ONE warning on the transition from
  // "no errors ever" to "errors counted" — the fingerprint of a link that is
  // dropping frames without having failed, which is otherwise invisible until
  // somebody reads the counters at the right moment.
  bool readBus(BusStatus& out);

 private:
  void logFrame(const char* dir, const cansimple::Frame& frame);

  static Diagnostics* _self;

  uint8_t  _target;
  uint64_t _seen_nodes = 0;

  logx::OnChange<bool> _errors_counting;   // bus/tx/rx error counters non-zero
  logx::OnChange<int>  _bus_state;         // controller state transitions
};

}  // namespace candiag
