// ============================================================================
//  axis_names.h — turning axis numbers into words.
//
//  Generated from ../include/axis_vocab.h, the firmware's own table. So a log
//  line says
//
//      log I AXIS state IDLE -> CLOSED_LOOP
//      log E AXIS error 0x0 -> 0x140 [MOTOR_FAILED|ENCODER_FAILED]
//
//  rather than leaving the reader to look 0x140 up in a header. Adding an error
//  bit to that table names it here, in the firmware, and on the GUI's CAN
//  Devices page at the same time.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "gvl/axis_io.h"   // the enums, generated from the same table

namespace axisnames {

const char* state(uint8_t axis_state);
const char* mode(uint8_t control_mode);

// Decode an error word into "[A|B|C]" (or "none") in `buf`. Unknown bits are
// reported as "+0xNNN" rather than dropped: a bit this build has no name for is
// precisely the one worth knowing about.
void errors(uint32_t err, char* buf, size_t len);

}  // namespace axisnames
