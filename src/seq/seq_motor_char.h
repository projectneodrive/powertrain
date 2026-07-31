// ============================================================================
//  seq_motor_char.h — commissioning SEQUENCE: measure the motor's phase
//  resistance and inductance.
//
//  Sequences are the odd ones out in this architecture: unlike a program, they
//  BLOCK for seconds while they drive the motor open-loop. They are invoked
//  from PRG_CONTROL, which means the COMMS task (CAN + control + telemetry)
//  stalls for the duration. That is acceptable only because they run with the
//  motor DISARMED, on an explicit operator request, on a bench.
// ============================================================================
#pragma once

namespace seq {

// Runs the R/L characterisation, or explains why it cannot. Always reports
// something: a request refused in silence used to look like a dead command.
//   safe : no fault latched and no e-stop
void motorCharacterise(bool safe);

} // namespace seq
