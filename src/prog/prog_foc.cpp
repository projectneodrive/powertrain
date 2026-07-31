// ============================================================================
//  prog_foc.cpp — the FOC loop, at 20 kHz.
// ============================================================================
#include "prog/prog_foc.h"
#include "io/io_motor.h"
#include "fb/fb_regen_clamp.h"
#include "gvl/gvl.h"
#include "config/plc_config.h"

namespace prog {

PrgFoc prgFoc;

// Telemetry publish divider: 20 kHz -> 1 kHz. Deliberately NOT
// MOTION_DOWNSAMPLE, which happens to be the same number today but means
// something else (how often SimpleFOC runs motion control); coupling them
// would silently change the telemetry rate if that were ever retuned.
static constexpr uint8_t TEL_DIV = FOC_TICK_HZ / 1000;

void PrgFoc::scan() {
  auto& motor = io::motor::motor;

  // NEVER overwrite motor.shaft_angle/shaft_velocity here: move() stores the
  // filtered multi-turn angle in them, and overwriting mixes two reference
  // frames into the telemetry. At rest we only refresh the sensor so pos/vel
  // stay live.
  if (!gvl::M.foc_ready || gvl::M.fault) {
    io::motor::foc_sensor.update();
  } else {
    motor.loopFOC();             // does sensor.update() internally
    motor.move(gvl::Q.active_target);

    // Regen derate: bound ONLY the torque that opposes rotation. See
    // fb/fb_regen_clamp.h for why this sits after move().
    float sp = motor.current_sp;
    fb::regenClamp(sp, motor.shaft_velocity, gvl::Q.regen_iq_limit);
    motor.current_sp = sp;
  }

  // Publish pos/vel at 1 kHz. PRG_FOC is the only reader of the sensor, so the
  // rest of the firmware only ever sees atomic floats — see gvl/gvl.h.
  if (++_tel_div >= TEL_DIV) {
    _tel_div = 0;
    gvl::IN.shaft_angle = io::motor::foc_sensor.getAngle();
    gvl::IN.shaft_vel   = io::motor::foc_sensor.getVelocity();
  }
}

} // namespace prog
