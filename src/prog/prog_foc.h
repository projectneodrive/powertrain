// ============================================================================
//  prog_foc.h — PROGRAM PRG_FOC. Bound to the EVENT task paced by TIM6 at
//  FOC_TICK_HZ (20 kHz). The hard real-time path.
// ============================================================================
#pragma once
#include "plc/plc_program.h"

namespace prog {

class PrgFoc : public plc::Program {
 public:
  const char* name() const override { return "PRG_FOC"; }
  void scan() override;

 private:
  uint8_t _tel_div = 0;
};

extern PrgFoc prgFoc;

} // namespace prog
