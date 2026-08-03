// ============================================================================
//  cansimple.h — ODrive CANSimple over the ESP32's TWAI controller.
//
//  The protocol layer, and nothing else: framing, payload packing, transmit,
//  receive. It knows what a frame is; it does not know what an axis is. The
//  application on top of it is lib/can_bridge.
//
//  WHAT MAKES THIS MORE THAN A TWAI WRAPPER is that the three query functions
//  below — firmwareAccepts(), firmwareBroadcasts(), cyclicPeriodMs() — are
//  generated from the FIRMWARE's own command table (../include/can_commands.h,
//  the same file its CAN driver is generated from). So this side of the bus can
//  answer, at compile time:
//
//    * may I send this command?          firmwareAccepts()      -> static_assert
//    * will this frame ever arrive?      firmwareBroadcasts()
//    * how often, so I can time out?     cyclicPeriodMs()
//
//  A command the firmware does not implement is a build error at the send site,
//  not a frame that vanishes into a bus nobody is decoding.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"

#include "bridge_config.h"
#include "can_ids.h"        // firmware-shared: odcan::Cmd + arbId/nodeOf/cmdOf

namespace cansimple {

// ---------------------------------------------------------------------------
//  A decoded frame
// ---------------------------------------------------------------------------
struct Frame {
  uint32_t id   = 0;    // raw arbitration id
  uint8_t  node = 0;    // (id >> 5) & 0x3F
  uint8_t  cmd  = 0;    // id & 0x1F
  uint8_t  len  = 0;
  uint8_t  data[8] = {};
};

// ---------------------------------------------------------------------------
//  Payload packing. CANSimple is little-endian, which is also the ESP32's and
//  the STM32's byte order, so memcpy is the encoding — but it is written out
//  rather than aliased through a cast, because type-punning a float through a
//  uint32_t* is undefined behaviour that GCC does miscompile at -O2.
// ---------------------------------------------------------------------------
inline void putF32(uint8_t* b, float v)     { memcpy(b, &v, sizeof(v)); }
inline void putU32(uint8_t* b, uint32_t v)  { memcpy(b, &v, sizeof(v)); }
inline float getF32(const uint8_t* b)       { float v;    memcpy(&v, b, sizeof(v)); return v; }
inline uint32_t getU32(const uint8_t* b)    { uint32_t v; memcpy(&v, b, sizeof(v)); return v; }

// ---------------------------------------------------------------------------
//  What the firmware implements — read straight out of its command table.
//
//  Each of these expands ../include/can_commands.h with one of the two macros
//  neutralised, folding the table into a single constant expression. They are
//  usable in static_assert, and cost nothing at run time.
// ---------------------------------------------------------------------------

// True if the firmware declares an RX handler for `cmd` — i.e. sending it is
// meaningful. Anything else it deliberately ignores.
constexpr bool firmwareAccepts(uint8_t cmd) {
  return false
#define CAN_RX(c, handler)                   || (cmd == (uint8_t)odcan::c)
#define CAN_TX_CYCLIC(c, period_ms, sender)
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX
      ;
}

// True if the firmware broadcasts `cmd` on a timer — i.e. it will arrive
// unprompted and the bridge needs a decoder for it.
constexpr bool firmwareBroadcasts(uint8_t cmd) {
  return false
#define CAN_RX(c, handler)
#define CAN_TX_CYCLIC(c, period_ms, sender)  || (cmd == (uint8_t)odcan::c)
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX
      ;
}

// Broadcast period of `cmd` in ms, or 0 if it is not cyclic. A sum rather than
// a chain of conditionals so it stays one expression (C++11 constexpr); safe
// because a command appears at most once in the cyclic list.
constexpr uint32_t cyclicPeriodMs(uint8_t cmd) {
  return 0
#define CAN_RX(c, handler)
#define CAN_TX_CYCLIC(c, period_ms, sender)  + (cmd == (uint8_t)odcan::c ? (uint32_t)(period_ms) : 0u)
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX
      ;
}

// Name of a command for logging, or "UNKNOWN". Also generated from the table,
// so it names exactly what the firmware implements — an "UNKNOWN" in a trace is
// therefore informative: that frame came from something other than our board's
// command set.
const char* cmdName(uint8_t cmd);

// ---------------------------------------------------------------------------
//  The link
// ---------------------------------------------------------------------------
class Link {
 public:
  // Called for every frame that goes out or comes in, when one is installed.
  // Lets lib/can_diag trace the bus without this layer depending on it.
  using FrameTap = void (*)(const char* dir, const Frame& frame, bool ok);

  explicit Link(uint8_t target_node) : _node(target_node) {}

  // Install and start the TWAI driver. Retries until it succeeds, printing
  // why: at this point there is nothing else worth doing, and a station that
  // came up with no bus is worse than one that says so every 500 ms.
  void begin();

  void onFrame(FrameTap tap) { _tap = tap; }

  uint8_t targetNode() const { return _node; }

  // Send `cmd` to the target node. The template parameter is what allows the
  // check: the command must be one the firmware actually handles.
  template <uint8_t CMD>
  bool send(const uint8_t* data = nullptr, uint8_t len = 0) {
    static_assert(firmwareAccepts(CMD),
                  "This command has no RX handler in the firmware's table "
                  "(include/can_commands.h) — the board would ignore the frame. "
                  "Add it there with a handler, or do not send it.");
    return sendRaw(CMD, data, len);
  }

  // Unchecked send. For tooling that must be able to probe a foreign node —
  // prefer send<CMD>() everywhere else.
  bool sendRaw(uint8_t cmd, const uint8_t* data, uint8_t len);

  // Pop one frame from the RX queue. Returns false when the queue is empty.
  bool receive(Frame& out);

  uint32_t txOkCount()   const { return _tx_ok; }
  uint32_t txFailCount() const { return _tx_fail; }
  uint32_t rxCount()     const { return _rx_count; }

 private:
  uint8_t  _node;
  FrameTap _tap = nullptr;
  uint32_t _tx_ok = 0, _tx_fail = 0, _rx_count = 0;
};

}  // namespace cansimple
