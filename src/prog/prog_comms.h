// ============================================================================
//  prog_comms.h — the two fieldbus PROGRAMs that bracket PRG_CONTROL in the
//  COMMS task, giving that task the classic PLC scan shape:
//
//      PRG_FIELDBUS_IN  ->  PRG_CONTROL  ->  PRG_TELEMETRY_OUT
//        read inputs         run logic        write outputs
// ============================================================================
#pragma once
#include "plc/plc_program.h"

namespace prog {

// Drains the CAN RX ring and applies every received command to the process
// image. Runs FIRST so a setpoint that arrived since the last scan is acted on
// in this one.
class PrgFieldbusIn : public plc::Program {
 public:
  const char* name() const override { return "PRG_FIELDBUS_IN"; }
  void scan() override;
};

// Publishes the axis telemetry into the process image, then sends whichever
// cyclic CAN frames are due. Runs LAST so what goes on the wire reflects this
// scan's result rather than the previous one's.
class PrgTelemetryOut : public plc::Program {
 public:
  const char* name() const override { return "PRG_TELEMETRY_OUT"; }
  void scan() override;
};

extern PrgFieldbusIn   prgFieldbusIn;
extern PrgTelemetryOut prgTelemetryOut;

} // namespace prog
