// ============================================================================
//  seq_hall_cal.h — commissioning SEQUENCE: measure the real hall transition
//  angles (see io/HallSensorSmoothVel.h and motor_config.h).
//
//  Blocks the calling task for ~10 s. Run with the motor DISARMED. Hall builds
//  only.
// ============================================================================
#pragma once
#include "config/motor_config.h"

#if SENSOR_TYPE == SENSOR_TYPE_HALL

namespace seq {

// Spins open-loop at an imposed electrical speed: the commanded angle (theta)
// IS the true angle. The mean residual per sector between theta and the
// reported angle (a uniform 60 deg grid) is that sector's placement error; the
// global mean is removed (it belongs in zero_electric_angle) and the rest is
// converted from electrical to mechanical.
//
// 'dir' (sector direction vs theta) is measured from the sector sequence, so it
// does not depend on initFOC's sensor_direction.
//
// Prints the six offsets for pasting into CFG_HALL_CAL_OFFSETS, applies them
// live, and clears gvl::M.calibrated to force a re-initFOC on the next arm (so
// zero_electric_angle is recomputed with the offsets active).
// Returns false if any sector was seen too rarely to trust.
bool hallCalibrate();

} // namespace seq

#endif // SENSOR_TYPE_HALL
