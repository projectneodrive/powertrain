// ============================================================================
//  state.cpp — the one and only definition of the shared state. See state.h.
// ============================================================================
#include "state.h"

namespace state {

FromFoc       foc;
FromSafety    safety;
FromControl   control;
AtBoot        at_boot;
Requests      req;
odcan::AxisIO axis;

}  // namespace state
