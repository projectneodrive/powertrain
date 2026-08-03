// ============================================================================
//  can_ids.h — the CANSimple arbitration ids and id arithmetic.
//
//  Split out of lib/odrive_can/odrive_can.h so that hosts which CANNOT compile
//  that header — it pulls in STM32_CAN — still speak the protocol from the same
//  list. The ESP32 control station in can_utilities/ includes this file
//  directly; before the split it carried a hand-copied enum, which is exactly
//  the kind of duplication the tables in this directory exist to remove.
//
//  Arbitration id = (node_id << 5) | cmd   (11-bit standard, little-endian).
//
//  Keep this file and can_commands.h straight, they answer different questions:
//    can_ids.h       what the ODrive CANSimple protocol DEFINES (vocabulary)
//    can_commands.h  which of those THIS firmware implements  (our subset)
//  A master may only usefully send what the second list contains; can_utilities
//  turns that into a compile-time check (see cansimple::firmwareAccepts).
// ============================================================================
#pragma once
#include <stdint.h>

namespace odcan {

// ---- CANSimple command ids (low 5 bits of the arbitration id) --------------
enum Cmd : uint8_t {
  CMD_HEARTBEAT             = 0x001,
  CMD_ESTOP                 = 0x002,
  CMD_GET_MOTOR_ERROR       = 0x003,
  CMD_GET_ENCODER_ERROR     = 0x004,
  CMD_GET_SENSORLESS_ERROR  = 0x005,
  CMD_SET_AXIS_NODE_ID      = 0x006,
  CMD_SET_AXIS_STATE        = 0x007,
  CMD_GET_ENCODER_ESTIMATES = 0x009,
  CMD_GET_ENCODER_COUNT     = 0x00A,
  CMD_SET_CONTROLLER_MODE   = 0x00B,
  CMD_SET_INPUT_POS         = 0x00C,
  CMD_SET_INPUT_VEL         = 0x00D,
  CMD_SET_INPUT_TORQUE      = 0x00E,
  CMD_SET_LIMITS            = 0x00F,
  CMD_GET_IQ                = 0x014,
  CMD_GET_SENSORLESS_EST    = 0x015,
  CMD_REBOOT                = 0x016,
  CMD_GET_BUS_VI            = 0x017,
  CMD_CLEAR_ERRORS          = 0x018,
  CMD_SET_LINEAR_COUNT      = 0x019,
  CMD_SET_POS_GAIN          = 0x01A,
  CMD_SET_VEL_GAINS         = 0x01B,
  CMD_GET_ADC_VOLTAGE       = 0x01C,
  CMD_GET_CONTROLLER_ERROR  = 0x01D,
};

// ---- Id arithmetic ---------------------------------------------------------
// Node ids are 6 bits and command ids 5, which together fill the 11-bit
// standard identifier exactly. Masking on decode matters: an 11-bit id from a
// node above 63 does not exist, but a *29-bit* extended frame from unrelated
// equipment sharing the bus does, and unmasked it would decode as a plausible
// node/command pair.
constexpr uint32_t arbId(uint8_t node, uint8_t cmd) {
  return ((uint32_t)node << 5) | (uint32_t)(cmd & 0x1F);
}
constexpr uint8_t nodeOf(uint32_t arb_id) { return (uint8_t)((arb_id >> 5) & 0x3F); }
constexpr uint8_t cmdOf(uint32_t arb_id)  { return (uint8_t)(arb_id & 0x1F); }

}  // namespace odcan
