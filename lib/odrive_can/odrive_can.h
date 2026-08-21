// ============================================================================
//  odrive_can.h  —  ODrive CANSimple protocol layer for the SimpleFOC firmware.
//
//  Speaks the ODrive "CANSimple" protocol so the existing DBC
//  (CAN/create_can_dbc.py) and odrivetool / any CAN master work unchanged.
//  Arbitration id = (node_id << 5) | cmd_id  (11-bit standard, little-endian).
//
//  This module is deliberately decoupled from SimpleFOC: it is a fieldbus I/O
//  driver that only reads/writes the shared AxisIO block of the process image
//  (include/axis_io.h). The control programs (src/prog/) bridge that block
//  to the BLDCMotor; neither side knows about the other. Runs on
//  pazi88/STM32_CAN (CAN1 on PB8/PB9).
// ============================================================================
#pragma once
#include <Arduino.h>
#include <STM32_CAN.h>
#include "can_ids.h"       // odcan::Cmd — shared with the can_utilities host tool
#include "axis_io.h"   // AxisIO + the axis state/mode/error vocabulary

namespace odcan {

// The command ids used to be declared here. They now live in include/can_ids.h
// so the ESP32 control station (can_utilities/) can include them without
// dragging in STM32_CAN — see that file's header for the id/vocabulary split.

// AxisState / ControlMode / AxisErrorBits and the AxisIO block itself now live
// in include/axis_io.h — they are process data shared with the control
// programs, not part of this wire protocol. Only the arbitration ids above are.

// ---- The CAN interface -----------------------------------------------------
class OdriveCAN {
public:
  explicit OdriveCAN(AxisIO& io) : _io(io) {}

  // hardware bring-up: node id, bit rate, and NVIC priority for the CAN IRQ.
  void begin(uint8_t node_id, uint32_t baud, uint8_t irq_prio);

  // drain the RX ring buffer and dispatch (call from the comms task, >=1 kHz).
  void poll();

  // send the cyclic telemetry frames that are due at time `now_ms`.
  void txCyclic(uint32_t now_ms);

  // TX/RX health counters for diagnostics (see main.cpp SerialTask). tx_fail
  // climbing means STM32_CAN::write() is rejecting frames at the hardware
  // level (e.g. BUS_OFF) -- this is otherwise silent, since send() ignores
  // write()'s return value on its own.
  uint32_t txOkCount()   const { return _tx_ok; }
  uint32_t txFailCount() const { return _tx_fail; }
  uint32_t rxCount()     const { return _rx_count; }

private:
  void dispatch(uint8_t cmd, const CAN_message_t& m);
  void send(uint8_t cmd, const uint8_t* d, uint8_t len);

  // RX handlers and cyclic senders, declared from the command table so the
  // table and this class can never disagree about what exists.
#define CAN_RX(cmd, handler)                  void handler(const uint8_t* b);
#define CAN_TX_CYCLIC(cmd, period_ms, sender) void sender();
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX

  // Number of cyclic slots, counted from the same table.
  static constexpr uint8_t TX_CYCLIC_COUNT = 0
#define CAN_RX(cmd, handler)
#define CAN_TX_CYCLIC(cmd, period_ms, sender) + 1
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX
      ;

  AxisIO&    _io;
  STM32_CAN  _can{CAN1, ALT, RX_SIZE_64, TX_SIZE_16};   // CAN1 on PB8/PB9
  uint8_t    _node = 0;
  uint32_t   _t_cyclic[TX_CYCLIC_COUNT] = {};   // last TX time per cyclic slot
  volatile uint32_t _tx_ok = 0, _tx_fail = 0, _rx_count = 0;
};

} // namespace odcan
