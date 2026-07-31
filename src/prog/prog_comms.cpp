// ============================================================================
//  prog_comms.cpp — see prog_comms.h.
// ============================================================================
#include "prog/prog_comms.h"
#include "io/io_can.h"
#include "io/io_motor.h"
#include "gvl/gvl.h"

using namespace odcan;

namespace prog {

PrgFieldbusIn   prgFieldbusIn;
PrgTelemetryOut prgTelemetryOut;

void PrgFieldbusIn::scan() {
  io::can::bus.poll();
}

void PrgTelemetryOut::scan() {
  auto& AX    = gvl::AXIS;
  auto& motor = io::motor::motor;

  // pos/vel are published by PRG_FOC (single writer), so reading them here is
  // atomic. They are in the SENSOR's convention; the sign correction to the
  // axis convention happens here.
  float sgn = (motor.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;
  AX.pos_rev = sgn * gvl::IN.shaft_angle / TWO_PI;
  AX.vel_rev = sgn * gvl::IN.shaft_vel   / TWO_PI;
  AX.vbus    = gvl::IN.vbus_filt;   // sampled/filtered by PRG_SAFETY

  if (gvl::M.foc_ready && gvl::IN.isense_ok) {
    AX.iq_setpoint = motor.current_sp;
    AX.iq_measured = motor.current.q;
    float p = motor.voltage.q * motor.current.q + motor.voltage.d * motor.current.d;
    AX.ibus = (AX.vbus > 1.0f) ? (p / AX.vbus) : 0.0f;
  } else {
    AX.iq_setpoint = 0.0f;
    AX.iq_measured = 0.0f;
    AX.ibus        = 0.0f;
  }
  AX.cur_state = gvl::M.foc_ready ? AXIS_CLOSED_LOOP : AXIS_IDLE;

  io::can::bus.txCyclic(millis());
}

} // namespace prog
