// ============================================================================
//  io_brake.cpp — see io_brake.h for the topology and the GVDD dependency.
//
//  CENTER-ALIGNED PWM (CMS=3), like the ODrive firmware. In edge-aligned mode,
//  at the counter's wrap, HI falls and LI rises at the SAME instant: the dead
//  time is only honoured on one edge and the other is a hard shoot-through.
//  Center-aligned, the counter passes each CCR twice, so the CCR4-CCR3 gap
//  produces the dead time on BOTH edges.
//
//  CH3 (LI, low FET)  in PWM mode 1: active while CNT <  CCR3
//  CH4 (HI, high FET) in PWM mode 2: active while CNT >= CCR4
//  With CCR3 < CCR4 the two drives never overlap.
// ============================================================================
#include "io/io_brake.h"
#include "config/hw_pinout.h"
#include "config/motor_config.h"

namespace io {
namespace brake {
namespace {

HardwareTimer *g_timer  = nullptr;
uint32_t       g_chanL  = 0;      // PIN_AUX_L -> LI
uint32_t       g_chanH  = 0;      // PIN_AUX_H -> HI
uint32_t       g_period = 0;      // ticks per half-period (= ARR+1)
uint32_t       g_dead   = 0;      // dead time in ticks
volatile float g_duty   = 0.0f;   // applied duty (telemetry)

// The two CCRs are ALWAYS written together. Preload (OC3PE/OC4PE, armed in
// init()) makes the pair load atomically at the update event: without it there
// is a one-period window where CCR3 is already the new value and CCR4 still the
// old one -> both FETs overlap = a short across the bus.
inline void applyCcr(uint32_t ccrL, uint32_t ccrH) {
  if (!g_timer) return;
  g_timer->setCaptureCompare(g_chanL, ccrL, TICK_COMPARE_FORMAT);
  g_timer->setCaptureCompare(g_chanH, ccrH, TICK_COMPARE_FORMAT);
}

// CCR = g_period => never reached (CNT max = ARR = g_period-1) => never active
// CCR3 = 0       => CNT < 0 is never true                      => LI never active
inline void stateOff() { applyCcr(0, g_period); }

void statePwm(float d) {
  int32_t mid = (int32_t)((float)g_period * (1.0f - d));  // CCR4 -> high-side duty = d
  int32_t lo  = mid - (int32_t)g_dead;                    // CCR3
  if (lo < 0)                  lo  = 0;
  if (mid > (int32_t)g_period) mid = (int32_t)g_period;
  applyCcr((uint32_t)lo, (uint32_t)mid);
}

} // namespace

void preInit() {
  pinMode(PIN_AUX_H, OUTPUT); digitalWrite(PIN_AUX_H, LOW);
  pinMode(PIN_AUX_L, OUTPUT); digitalWrite(PIN_AUX_L, LOW);
}

void off() {
  g_duty = 0.0f;
  stateOff();
}

void init() {
  PinName pl = digitalPinToPinName(PIN_AUX_L);
  PinName ph = digitalPinToPinName(PIN_AUX_H);
  TIM_TypeDef *inst = (TIM_TypeDef *)pinmap_peripheral(pl, PinMap_TIM);
  g_chanL = STM_PIN_CHANNEL(pinmap_function(pl, PinMap_TIM));
  g_chanH = STM_PIN_CHANNEL(pinmap_function(ph, PinMap_TIM));
  g_timer = new HardwareTimer(inst);
  g_timer->setOverflow(CFG_BRAKE_PWM_HZ, HERTZ_FORMAT);   // ARR edge-aligned

  // Dead time in ticks, derived from the frequency the chosen prescaler
  // actually produced (not from a hard-coded value).
  uint32_t psc    = g_timer->getPrescaleFactor();
  uint32_t tickHz = g_timer->getTimerClkFreq() / (psc ? psc : 1);
  g_dead = (uint32_t)(((uint64_t)tickHz * CFG_BRAKE_DEADTIME_NS) / 1000000000ULL);
  if (g_dead < 1) g_dead = 1;

  TIM_TypeDef *T = g_timer->getHandle()->Instance;
  // Center-aligned, the counter runs up and down: 2x more ticks per PWM period.
  // Halve (ARR+1) to keep CFG_BRAKE_PWM_HZ.
  uint32_t period_edge = T->ARR + 1u;
  T->ARR = (period_edge / 2u) - 1u;
  MODIFY_REG(T->CR1, TIM_CR1_CMS, TIM_CR1_CMS);   // CMS = 0b11
  g_period = T->ARR + 1u;

  stateOff();                                     // safe OFF BEFORE enabling the AF
  g_timer->setMode(g_chanL, TIMER_OUTPUT_COMPARE_PWM1, PIN_AUX_L);
  g_timer->setMode(g_chanH, TIMER_OUTPUT_COMPARE_PWM2, PIN_AUX_H);

  // Comparator preload: CCR3/CCR4 change together, at the update event.
  T->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  T->CR1   |= TIM_CR1_ARPE;
  T->EGR    = TIM_EGR_UG;                         // reload ARR/PSC/CCR

  stateOff();                                     // setMode may have reset CCR to 0
  g_timer->resume();
}

void setDuty(float d) {
  d = constrain(d, 0.0f, (float)CFG_BRAKE_MAX_DUTY);
  g_duty = d;
  if (d <= 0.0f) stateOff();
  else           statePwm(d);
}

float duty() { return g_duty; }

} // namespace brake
} // namespace io
