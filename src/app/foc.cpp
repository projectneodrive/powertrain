// ============================================================================
//  foc.cpp — the FOC loop, at 20 kHz. See app.h.
// ============================================================================
#include "app.h"

#include "config/tasks_config.h"
#include "io/io_motor.h"
#include "state.h"
#include "util/timers.h"

namespace app {
namespace foc {
namespace {

// Telemetry publish divider: 20 kHz -> 1 kHz. Deliberately NOT
// MOTION_DOWNSAMPLE, which happens to be the same number today but means
// something else (how often SimpleFOC runs motion control); coupling them would
// silently change the telemetry rate if that were ever retuned.
constexpr uint8_t TEL_DIV = FOC_TICK_HZ / 1000;
uint8_t s_tel_div = 0;

// Apply the bus regen derate to the torque command, and ONLY to the part of it
// that opposes rotation (sp * vel < 0, i.e. braking). Motoring torque is never
// reduced: it does not charge the bus, so limiting it would cost performance
// for nothing.
//
// Called after move(), every tick. That placement matters: current_sp persists
// between two move() calls (MOTION_DOWNSAMPLE = 20), so clamping after move()
// covers each of the following loopFOC() calls too.
//
// In the voltage-torque fallback current_sp is in volts rather than amps. The
// clamp stays homogeneous because current_limit bounds volts there as well.
inline void regenClamp(float &current_sp, float velocity, float limit) {
  if (current_sp * velocity < 0.0f)
    current_sp = util::limit(-limit, current_sp, limit);
}

}  // namespace

void update() {
  auto &motor = io::motor::motor;

  // NEVER overwrite motor.shaft_angle/shaft_velocity here: move() stores the
  // filtered multi-turn angle in them, and overwriting mixes two reference
  // frames into the telemetry. At rest we only refresh the sensor so pos/vel
  // stay live.
  if (!state::control.foc_ready || state::safety.fault) {
    io::motor::foc_sensor.update();
  } else {
    motor.loopFOC();                        // does sensor.update() internally
    motor.move(state::control.active_target);

    float sp = motor.current_sp;
    regenClamp(sp, motor.shaft_velocity, state::safety.regen_iq_limit);
    motor.current_sp = sp;
  }

  // Publish at 1 kHz. This module is the only reader of the sensor and the only
  // writer of the four dq terms, so everything below is coherent here and is
  // published as independent atomic floats — see the pairs rule in state.h.
  if (++s_tel_div >= TEL_DIV) {
    s_tel_div = 0;
    state::foc.shaft_angle = io::motor::foc_sensor.getAngle();
    state::foc.shaft_vel   = io::motor::foc_sensor.getVelocity();
    state::foc.iq_measured = motor.current.q;
    state::foc.iq_setpoint = motor.current_sp;
    // Electrical power, computed HERE because reading these four fields from
    // another task is a torn read against a 20 kHz writer.
    state::foc.p_elec = motor.voltage.q * motor.current.q +
                        motor.voltage.d * motor.current.d;
  }
}

}  // namespace foc
}  // namespace app
