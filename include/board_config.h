// ============================================================================
//  board_config.h  —  Umbrella header kept for compatibility. The single source
//  of truth is now split by concern:
//
//    config/hw_pinout.h     pins and hardware topology (what is wired where)
//    config/motor_config.h  motor, power, limits, calibration, controller gains
//    config/tasks_config.h    PLC task rates, priorities, stacks, NVIC policy
//
//  New code should include the specific header it needs. The standalone bench
//  sketches in test/ include this one.
// ============================================================================
#pragma once

#include "config/hw_pinout.h"
#include "config/motor_config.h"
#include "config/tasks_config.h"
