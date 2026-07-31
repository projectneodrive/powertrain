// ============================================================================
//  io_can.h — I/O module for the CANSimple fieldbus.
//
//  A thin binding: it owns the single OdriveCAN instance and maps it onto the
//  process image's axis block (gvl::AXIS). The protocol itself lives in
//  lib/odrive_can; the commands it answers are declared in
//  include/can_commands.h.
// ============================================================================
#pragma once
#include "odrive_can.h"

namespace io {
namespace can {

extern odcan::OdriveCAN bus;

// Bring up CAN1 and print the node/bit-rate line.
void init();

} // namespace can
} // namespace io
