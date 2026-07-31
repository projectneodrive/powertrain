// ============================================================================
//  prog_control.h — PROGRAM PRG_CONTROL: the axis state machine. Runs at 1 kHz
//  in the COMMS task, between PRG_FIELDBUS_IN and PRG_TELEMETRY_OUT, so a
//  setpoint that arrives this cycle is acted on this cycle.
// ============================================================================
#pragma once
#include "plc/plc_program.h"

namespace prog {

class PrgControl : public plc::Program {
 public:
  const char* name() const override { return "PRG_CONTROL"; }
  void init() override;
  void scan() override;

 private:
  void applyPendingGains();
  bool runPendingSequence(bool safe);   // true if a sequence ran (skip this scan)
  void updateSetpoint();
};

extern PrgControl prgControl;

} // namespace prog
