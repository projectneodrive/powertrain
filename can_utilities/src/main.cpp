/* ===========================================================================
 *  can_utilities — ESP32 CANSimple control station for the Neodrive board.
 *
 *  This file is the whole program. Everything it does lives in lib/:
 *
 *    lib/cansimple    the CANSimple protocol over the ESP32's TWAI controller
 *    lib/can_bridge   the axis, the console, the telemetry line
 *    lib/can_diag     bus tracing, alerts and health counters
 *    lib/pot_input    the potentiometer, read as a velocity joystick
 *
 *  The command set, the telemetry channels, the node id and the bit rate are
 *  NOT defined in this project. They are compiled from the firmware's own
 *  tables in ../include/ so the two ends of the bus cannot drift apart —
 *  see README.md, and platformio.ini for how that include path is set up.
 * ===========================================================================*/

#include <Arduino.h>
#include "can_bridge.h"

void setup() {
  bridge::station.begin();
}

void loop() {
  bridge::station.poll();
}
