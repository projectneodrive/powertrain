// ============================================================================
//  io_can.cpp — see io_can.h.
// ============================================================================
#include "io/io_can.h"
#include "state.h"
#include "config/motor_config.h"
#include "config/tasks_config.h"

namespace io {
namespace can {

odcan::OdriveCAN bus(state::axis);

void init() {
  bus.begin(CFG_CAN_NODE_ID, CFG_CAN_BAUD, NVIC_PRIO_RTOS_SAFE);
  Serial.print("CAN up: node "); Serial.print(CFG_CAN_NODE_ID);
  Serial.print(" @ "); Serial.print(CFG_CAN_BAUD); Serial.println(" bps");
}

} // namespace can
} // namespace io
