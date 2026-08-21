// ============================================================================
//  comms.cpp — the CAN boundary. See app.h.
//
//  Pure transport: drain the bus into the axis block, publish the axis block
//  back out. It reads state.h and never touches the motor object — everything
//  it publishes was made coherent by the module that owns it.
// ============================================================================
#include "app.h"

#include "io/io_can.h"
#include "io/io_motor.h"
#include "state.h"

namespace app {
namespace comms {

void readFieldbus() {
  io::can::bus.poll();
}

void publishTelemetry() {
  auto &AX = state::axis;

  // pos/vel are published by foc as independent atomic floats, so reading them
  // here is safe. They are in the SENSOR's convention; the sign correction to
  // the axis convention happens here. sensor_direction is written once during
  // initFOC and read as a single byte.
  float sgn = (io::motor::motor.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;
  AX.pos_rev = sgn * state::foc.shaft_angle / TWO_PI;
  AX.vel_rev = sgn * state::foc.shaft_vel   / TWO_PI;
  AX.vbus    = state::safety.vbus_filt;

  if (state::control.foc_ready && state::at_boot.isense_ok) {
    AX.iq_setpoint = state::foc.iq_setpoint;
    AX.iq_measured = state::foc.iq_measured;
    // p_elec is computed by foc, where all four dq terms are coherent. Computing
    // it here would be a torn 4-field read against a 20 kHz writer.
    AX.ibus = (AX.vbus > 1.0f) ? (state::foc.p_elec / AX.vbus) : 0.0f;
  } else {
    AX.iq_setpoint = 0.0f;
    AX.iq_measured = 0.0f;
    AX.ibus        = 0.0f;
  }
  AX.cur_state = state::control.foc_ready ? odcan::AXIS_CLOSED_LOOP : odcan::AXIS_IDLE;

  io::can::bus.txCyclic(millis());
}

}  // namespace comms
}  // namespace app
