// ============================================================================
//  comms.cpp — the CAN boundary. See app.h.
//
//  Pure transport: drain the bus into the axis block, publish the axis block
//  back out. It reads state.h and never touches the motor object — everything
//  it publishes was made coherent by the module that owns it.
// ============================================================================
#include "app.h"

#include "config/motor_config.h"
#include "io/io.h"
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
  //
  // /!\ PROJECT DEVIATION from ODrive Get_Encoder_Estimates (0x009): pos and vel
  // are referred to the CRANK, not the motor — motor quantity / CFG_GEAR_RATIO.
  // That is the number a rider means by "cadence", and the whole reason the gear
  // ratio exists in the config.
  //
  // The asymmetry this creates is deliberate and worth knowing: the velocity
  // SETPOINT ('V', Set_Input_Vel) and CFG_VEL_LIMIT stay on the MOTOR side,
  // because that is the frame the FOC controls in. So commanding V15 and reading
  // back vel = 0.21 rad/s is correct, not a bug — it is 15 rad/s at the motor
  // seen through a 72:1 box.
  float sgn = (io::motor::motor.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;
  const float kPedal = 1.0f / (TWO_PI * CFG_GEAR_RATIO);
  AX.pos_rev = sgn * state::foc.shaft_angle * kPedal;
  AX.vel_rev = sgn * state::foc.shaft_vel   * kPedal;
  AX.vbus    = state::safety.vbus_filt;

  if (state::control.foc_ready && state::at_boot.isense_ok) {
    AX.iq_setpoint = state::foc.iq_setpoint;
    AX.iq_measured = state::foc.iq_measured;
    // p_elec is computed by foc, where all four dq terms are coherent. Computing
    // it here would be a torn 4-field read against a 20 kHz writer.
    AX.ibus = (AX.vbus > 1.0f) ? (state::foc.p_elec / AX.vbus) : 0.0f;

    // Where the braking energy goes. p_elec > 0 is the motor DRAWING from the
    // bus, so under regen it is negative and -p_elec is what arrives. Subtract
    // what the resistor burns and the remainder is the source's: p_batt > 0
    // means the pack is charging. p_res comes from app::safety, which owns the
    // chopper and is therefore the only module that can state its duty
    // coherently with the bus voltage it was applied at.
    AX.p_res  = state::safety.p_res;
    AX.p_batt = -state::foc.p_elec - AX.p_res;
    AX.i_batt = (AX.vbus > 1.0f) ? (AX.p_batt / AX.vbus) : 0.0f;
  } else {
    AX.iq_setpoint = 0.0f;
    AX.iq_measured = 0.0f;
    AX.ibus        = 0.0f;
    AX.p_res       = 0.0f;
    AX.p_batt      = 0.0f;
    AX.i_batt      = 0.0f;
  }
  AX.cur_state = state::control.foc_ready ? odcan::AXIS_CLOSED_LOOP : odcan::AXIS_IDLE;

  io::can::bus.txCyclic(millis());
}

}  // namespace comms
}  // namespace app
