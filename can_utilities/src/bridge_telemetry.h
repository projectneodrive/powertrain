// ============================================================================
//  bridge_telemetry.h — the "key=value" line the host GUI reads.
//
//  The point of this file is that GUI/serial_plotter_wasm (and
//  serial_plotter_fast.py) can be plugged into EITHER the board's own USB port
//  or this station's, and see the same stream. That only holds if the two
//  emitters agree on the channel list, so they are generated from one file:
//  ../include/telemetry_schema.h, which the firmware and the GUI also compile.
//
//  WHERE THE THREE CONSUMERS DIFFER. The schema carries a firmware-side value
//  expression per channel (gvl::AXIS.iq_measured and such). The GUI discards
//  it. This station cannot: it has no gvl:: at all, only what came back over
//  CAN. So each channel needs an accessor here saying where the bridge gets
//  that number from — and a channel the bridge CANNOT source says so by
//  returning false, and is then simply left out of the line rather than sent as
//  a fabricated zero.
//
//  Adding a channel to the schema therefore fails THIS link until somebody
//  decides which of those two it is. That is the whole reason the accessors are
//  declared by macro instead of being a hand-written table.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "bridge_axis.h"
#include "bridge_state.h"
#include "cansimple.h"    // the link + its bus diagnostics (candiag::)

namespace bridge {

// One accessor per telemetry channel, named after its wire key.
//
//   returns true  -> `out` holds the value, the channel is emitted
//   returns false -> the bridge has no way to know this one; it is omitted and
//                    the GUI keeps showing its last (or zero) value
//
// Defined in bridge_telemetry.cpp. An undefined one is a link error.
namespace channel {
#define TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr) \
  bool key(const State& s, float& out);
#define TELEMETRY_CHANNEL_HALL TELEMETRY_CHANNEL
#include "telemetry_schema.h"
#undef TELEMETRY_CHANNEL_HALL
#undef TELEMETRY_CHANNEL
}  // namespace channel

// Print one telemetry line, no faster than BRIDGE_TELEMETRY_MS. Returns true
// when a line was actually emitted.
bool emitTelemetry(uint32_t now_ms, State& state, const cansimple::Link& link,
                   bool link_fresh);

// ---------------------------------------------------------------------------
//  The `can ...` line — the CAN Devices page's data feed.
//
//      can node=0 link=1 hb_age=8 bus=1 axis=8 mode=2 axis_err=0x0 ...
//
//  This is the other half of the answer to an unreadable monitor pane. Node id,
//  link state, the four error words and every bus counter are STATE: they are
//  only meaningful as a live table, and printing them as prose every couple of
//  seconds is what made the log unreadable in the first place. They go out here
//  once a second, on a line the GUI routes to a page and never to the monitor.
//
//  Key/value, hex where an operator reads hex. The GUI keeps the raw tokens, so
//  `axis_err=0x140` survives as text and is decoded with the same axis_vocab.h
//  table the firmware and this station use.
// ---------------------------------------------------------------------------
// `state` and `axis` are non-const because the line CONSUMES two high-water
// marks (worst scan, worst heartbeat gap): reporting and resetting has to be
// one step, or two readers each see a fraction of the interval.
bool emitCanStatus(uint32_t now_ms, State& state, Axis& axis,
                   const cansimple::Link& link, candiag::Diagnostics& diag);

}  // namespace bridge
