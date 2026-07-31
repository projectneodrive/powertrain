// ============================================================================
//  io_motor.h — I/O module owning the SimpleFOC object graph: the 6-PWM driver,
//  the motor, the position sensor chain and the low-side current sense.
//
//  These stay public because the control programs legitimately drive them
//  (motor.move(), motor.loopFOC(), the PID members) and the console reports
//  them. What this module encapsulates is their CONSTRUCTION and WIRING — which
//  sensor is compiled in, what links to what, and the bring-up order — so that
//  a program never has to know whether a hall or a quadrature encoder is fitted.
// ============================================================================
#pragma once
#include <SimpleFOC.h>
#include "config/hw_pinout.h"
#include "config/motor_config.h"
#include "io/HallSensorSmoothVel.h"
#include "io/HybridSensor.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/smoothing/SmoothingSensor.h"

namespace io {
namespace motor {

extern BLDCDriver6PWM      driver;
extern BLDCMotor           motor;
extern LowsideCurrentSense current_sense;

#if SENSOR_TYPE == SENSOR_TYPE_HALL
extern HallSensorSmoothVel sensor;
extern SmoothingSensor     smooth;
extern HybridSensor        hybrid;
#else
extern STM32HWEncoder      sensor;
#endif

// The sensor the FOC actually reads. Aliases the last stage of the chain above,
// so programs never care how many wrappers are in front of it.
extern Sensor& foc_sensor;

// Full bring-up: sensor, driver, current sense, motor parameters and PID
// defaults. Prints the same boot lines as before. Publishes gvl::IN.isense_ok.
void init();

// Arm the power stage. The DRV8301 needs a settling delay before its SPI is
// reliable again, hence the delay before re-asserting the gain.
void enableStage();

// The ADC instance the current sense took, or nullptr if it never came up.
// io_vbus uses this to pick the OTHER ADC — see hw_pinout.h for why sharing one
// is not an option.
ADC_TypeDef* currentSenseAdc();

} // namespace motor
} // namespace io
