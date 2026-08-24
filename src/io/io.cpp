// ============================================================================
//  io.cpp — implementations for io.h, in the same order.
// ============================================================================
#include "io/io.h"

#include <SPI.h>
#include "config/hw_pinout.h"
#include "config/motor_config.h"
#include "io/io_motor.h"
#include "state.h"
#include "config/tasks_config.h"

// ============================================================================
//  io_gate.cpp — see io_gate.h.
// ============================================================================

namespace io {
namespace gate {
namespace {
SPIClass spi3(PIN_DRV_MOSI, PIN_DRV_MISO, PIN_DRV_SCK);
}

DRV8301 drv(spi3, PIN_M0_CS);

void preInit() {
  pinMode(PIN_M1_CS, OUTPUT); digitalWrite(PIN_M1_CS, HIGH); // Disable unused M1 SPI
  pinMode(PIN_N_FAULT, INPUT_PULLUP);

  // DRV8301 hardware reset: it latches its configuration out of the EN_GATE
  // rising edge, so the low pulse is what guarantees a known starting state.
  pinMode(PIN_EN_GATE, OUTPUT);
  digitalWrite(PIN_EN_GATE, LOW);  delay(50);
  digitalWrite(PIN_EN_GATE, HIGH); delay(50);
}

void init() {
  drv.begin();
  bool gain_ok = drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
  Serial.print("DRV8301 status1=0x"); Serial.print(drv.status1(), HEX);
  Serial.print(" gain_set="); Serial.println(gain_ok ? "OK" : "FAIL(check SPI)");
}

void setGain() {
  drv.setGain(DRV8301::gainFromVpV(CFG_DRV_GAIN));
}

} // namespace gate
} // namespace io

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

// ============================================================================
//  io_vbus.cpp — see io_vbus.h.
// ============================================================================

namespace io {
namespace vbus {
namespace {
ADC_HandleTypeDef g_adc = {};
}

bool init() {
  // Take whichever ADC the current sense did NOT claim, so the injected
  // conversions driven by TIM1 can never interact with ours.
  ADC_TypeDef *cs_inst = motor::currentSenseAdc();
  ADC_TypeDef *inst    = (cs_inst == ADC1) ? ADC2 : ADC1;

  if (inst == ADC1) { __HAL_RCC_ADC1_CLK_ENABLE(); }
  else              { __HAL_RCC_ADC2_CLK_ENABLE(); }
  pinmap_pinout(digitalPinToPinName(PIN_VBUS), PinMap_ADC);  // PA6 analog

  g_adc.Instance = inst;
  // Same prescaler as the library uses. This register is COMMON to both ADCs —
  // do not deviate from it.
  g_adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  g_adc.Init.Resolution            = ADC_RESOLUTION_12B;
  g_adc.Init.ScanConvMode          = DISABLE;
  g_adc.Init.ContinuousConvMode    = DISABLE;
  g_adc.Init.DiscontinuousConvMode = DISABLE;
  g_adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  g_adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  g_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  g_adc.Init.NbrOfConversion       = 1;
  g_adc.Init.DMAContinuousRequests = DISABLE;
  g_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&g_adc) != HAL_OK) { g_adc.Instance = nullptr; return false; }

  ADC_ChannelConfTypeDef c = {};
  c.Channel      = ADC_CHANNEL_6;             // PA6 = ADC12_IN6
  c.Rank         = 1;
  c.SamplingTime = ADC_SAMPLETIME_480CYCLES;  // high-impedance divider
  c.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&g_adc, &c) != HAL_OK) { g_adc.Instance = nullptr; return false; }

  Serial.print("Vbus ADC: dedicated ADC");
  Serial.print(inst == ADC1 ? 1 : 2);
  Serial.print(" (current sense on ADC");
  Serial.print(cs_inst == ADC1 ? "1" : (cs_inst == ADC2 ? "2" : "?"));
  Serial.println(")");
  return true;
}

float readRaw() {
  if (!g_adc.Instance) return -1.0f;
  if (HAL_ADC_Start(&g_adc) != HAL_OK) return -1.0f;
  float v = -1.0f;
  if (HAL_ADC_PollForConversion(&g_adc, 1) == HAL_OK)
    v = HAL_ADC_GetValue(&g_adc) * (3.3f / 4096.0f);
  HAL_ADC_Stop(&g_adc);
  return v;
}

} // namespace vbus
} // namespace io

// ============================================================================
//  io_can.cpp — see io_can.h.
// ============================================================================

namespace io {
namespace can {

odcan::OdriveCAN bus(state::axis);

void init() {
  bus.begin(CFG_CAN_NODE_ID, CFG_CAN_BAUD, NVIC_PRIO_RTOS_SAFE);
  Serial.print("CAN up: node "); Serial.print(CFG_CAN_NODE_ID);
  Serial.print(" @ "); Serial.print(CFG_CAN_BAUD); Serial.println(" bps");
}

} // namespace can
} // namespace io

// ============================================================================
//  io_console.cpp — see io_console.h.
// ============================================================================

namespace io {
namespace console {
namespace {
char    g_buf[24];
uint8_t g_idx = 0;
}

const char* poll() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      uint8_t len = g_idx;
      g_buf[len] = '\0';
      g_idx = 0;
      if (len > 0) return g_buf;   // empty lines are swallowed, keep draining
    } else if (g_idx < sizeof(g_buf) - 1) {
      g_buf[g_idx++] = c;
    }
  }
  return nullptr;
}

void ackFloat(const char* tag, const char* field, float oldv, float newv,
              uint8_t prec, const char* unit) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.print(field); Serial.print(' ');
  Serial.print(oldv, prec);
  Serial.print(" -> ");
  Serial.print(newv, prec);
  if (unit) { Serial.print(' '); Serial.print(unit); }
  Serial.println();
}

void ackInt(const char* tag, const char* field, long oldv, long newv) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.print(field); Serial.print(' ');
  Serial.print(oldv);
  Serial.print(" -> ");
  Serial.println(newv);
}

void ackMsg(const char* tag, const char* message) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.println(message);
}

} // namespace console
} // namespace io
