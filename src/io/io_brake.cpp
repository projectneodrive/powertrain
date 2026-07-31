// ============================================================================
//  brake.cpp — voir brake.h pour la topologie et la dépendance à GVDD.
//
//  PWM CENTER-ALIGNED (CMS=3), comme le firmware ODrive. En edge-aligned, au
//  repli du compteur, HI descend et LI monte au MÊME instant : le temps mort
//  n'est ménagé que sur un seul front et l'autre est un shoot-through franc.
//  En center-aligned le compteur passe deux fois par chaque CCR, donc l'écart
//  CCR4-CCR3 produit le temps mort sur LES DEUX fronts.
//
//  CH3 (LI, FET bas)  en PWM mode 1 : actif tant que CNT <  CCR3
//  CH4 (HI, FET haut) en PWM mode 2 : actif tant que CNT >= CCR4
//  Avec CCR3 < CCR4 les deux commandes ne se recouvrent jamais.
// ============================================================================
#include "brake.h"
#include "board_config.h"

namespace brake {
namespace {

HardwareTimer *g_timer  = nullptr;
uint32_t       g_chanL  = 0;      // PIN_AUX_L -> LI
uint32_t       g_chanH  = 0;      // PIN_AUX_H -> HI
uint32_t       g_period = 0;      // ticks par demi-période (= ARR+1)
uint32_t       g_dead   = 0;      // temps mort en ticks
volatile float g_duty   = 0.0f;   // duty appliqué (télémétrie)
bool           g_engaged = false; // état de l'hystérésis du chopper

// Les deux CCR sont TOUJOURS écrits ensemble. Le preload (OC3PE/OC4PE, armé
// dans init()) fait que la paire est chargée atomiquement à l'update event :
// sans lui, il existe une fenêtre d'une période où CCR3 est déjà le nouveau et
// CCR4 encore l'ancien -> recouvrement des deux FETs = court-circuit du bus.
inline void applyCcr(uint32_t ccrL, uint32_t ccrH) {
  if (!g_timer) return;
  g_timer->setCaptureCompare(g_chanL, ccrL, TICK_COMPARE_FORMAT);
  g_timer->setCaptureCompare(g_chanH, ccrH, TICK_COMPARE_FORMAT);
}

// CCR = g_period => jamais atteint (CNT max = ARR = g_period-1) => jamais actif
// CCR3 = 0       => CNT < 0 jamais vrai                         => LI jamais actif
inline void stateOff() { applyCcr(0, g_period); }

void statePwm(float d) {
  int32_t mid = (int32_t)((float)g_period * (1.0f - d));  // CCR4 -> duty haut = d
  int32_t lo  = mid - (int32_t)g_dead;                    // CCR3
  if (lo < 0)                 lo  = 0;
  if (mid > (int32_t)g_period) mid = (int32_t)g_period;
  applyCcr((uint32_t)lo, (uint32_t)mid);
}

} // namespace

void off() {
  g_duty    = 0.0f;
  g_engaged = false;
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

  // Temps mort en ticks, déduit de la fréquence réellement obtenue par le
  // prescaler choisi ci-dessus (pas d'une valeur codée en dur).
  uint32_t psc    = g_timer->getPrescaleFactor();
  uint32_t tickHz = g_timer->getTimerClkFreq() / (psc ? psc : 1);
  g_dead = (uint32_t)(((uint64_t)tickHz * CFG_BRAKE_DEADTIME_NS) / 1000000000ULL);
  if (g_dead < 1) g_dead = 1;

  TIM_TypeDef *T = g_timer->getHandle()->Instance;
  // En center-aligned le compteur fait montant+descendant : 2x plus de ticks
  // par période PWM. On divise (ARR+1) par 2 pour conserver CFG_BRAKE_PWM_HZ.
  uint32_t period_edge = T->ARR + 1u;
  T->ARR = (period_edge / 2u) - 1u;
  MODIFY_REG(T->CR1, TIM_CR1_CMS, TIM_CR1_CMS);   // CMS = 0b11
  g_period = T->ARR + 1u;

  stateOff();                                     // OFF sûr AVANT d'activer l'AF
  g_timer->setMode(g_chanL, TIMER_OUTPUT_COMPARE_PWM1, PIN_AUX_L);
  g_timer->setMode(g_chanH, TIMER_OUTPUT_COMPARE_PWM2, PIN_AUX_H);

  // Preload des comparateurs : CCR3/CCR4 changent ensemble, à l'update event.
  T->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  T->CR1   |= TIM_CR1_ARPE;
  T->EGR    = TIM_EGR_UG;                         // recharge ARR/PSC/CCR

  stateOff();                                     // setMode a pu remettre CCR à 0
  g_timer->resume();
}

void update(float vbus, bool stage_active) {
  // Défaut sûr : étage désarmé/en faute, ou mesure de bus invalide -> OFF.
  // Étage désarmé, GVDD est absente de toute façon : commuter n'aurait aucun
  // effet, mais on garde les registres dans un état défini.
  if (!stage_active || vbus <= 0.0f) { off(); return; }

  // Hystérésis : on s'engage au-dessus de VBUS_ON, on ne relâche qu'en
  // repassant sous VBUS_OFF. Une fois engagé le duty est calculé à partir de
  // VBUS_OFF (et non de VBUS_ON) : sinon il serait négatif — donc nul — dans
  // toute la bande d'hystérésis, et le chopper se contenterait de battre
  // autour de VBUS_ON au lieu de tenir le bus dans la bande.
  if (!g_engaged) {
    if (vbus <= CFG_BRAKE_VBUS_ON) { g_duty = 0.0f; stateOff(); return; }
    g_engaged = true;
  } else if (vbus < CFG_BRAKE_VBUS_OFF) {
    g_engaged = false;
    g_duty    = 0.0f;
    stateOff();
    return;
  }

  float d = (vbus - CFG_BRAKE_VBUS_OFF) * CFG_BRAKE_GAIN;
  d = constrain(d, 0.0f, (float)CFG_BRAKE_MAX_DUTY);
  g_duty = d;
  if (d <= 0.0f) stateOff();
  else           statePwm(d);
}

float duty() { return g_duty; }

} // namespace brake
