// ============================================================================
//  io_can.cpp — see io_can.h.
// ============================================================================
#include "io/io_can.h"
#include "gvl/gvl.h"
#include "config/motor_config.h"
#include "config/plc_config.h"

namespace io {
namespace can {

odcan::OdriveCAN bus(gvl::AXIS);

void init() {
  bus.begin(CFG_CAN_NODE_ID, CFG_CAN_BAUD, NVIC_PRIO_RTOS_SAFE);
  Serial.print("CAN up: node "); Serial.print(CFG_CAN_NODE_ID);
  Serial.print(" @ "); Serial.print(CFG_CAN_BAUD); Serial.println(" bps");
}

} // namespace can
} // namespace io
