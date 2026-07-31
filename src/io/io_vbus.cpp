// ============================================================================
//  io_vbus.cpp — see io_vbus.h.
// ============================================================================
#include "io/io_vbus.h"
#include "io/io_motor.h"
#include "config/hw_pinout.h"

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
