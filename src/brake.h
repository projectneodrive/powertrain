// ============================================================================
//  brake.h — chopper de dissipation sur la résistance de freinage (bornes AUX).
//
//  Matériel : demi-pont AUX du DRV8301/ODESC — gate driver LM5109B (U7) +
//  NTMFS5C628N (IC15//IC16 côté haut, IC13//IC14 côté bas). La résistance est
//  câblée entre le point milieu (JP2.2 / TP13) et la masse (JP2.1 / TP12) :
//  c'est donc le FET HAUT qui dissipe.
//
//  /!\ DÉPENDANCE MATÉRIELLE CRITIQUE : le VDD du LM5109B est alimenté par
//  GVDD, le régulateur de grille interne du DRV8301, présent uniquement quand
//  EN_GATE est haut. Le frein est donc PHYSIQUEMENT INOPÉRANT étage désarmé —
//  ce n'est pas une décision logicielle, c'est le câblage de la carte.
//  Conséquence directe : aucune protection contre les surtensions n'existe
//  quand l'étage est désarmé (moteur entraîné mécaniquement à l'arrêt, par
//  exemple). TODO: si une protection permanente est nécessaire, maintenir
//  EN_GATE haut en permanence et couper le moteur autrement (BDTR.MOE de TIM1
//  à 0 met les six grilles moteur en Hi-Z sans toucher à EN_GATE, donc sans
//  perdre GVDD).
//
//  Le module ne touche QUE TIM2/CH3/CH4. TIM1 (PWM moteur), TIM3 (capteur) et
//  TIM6 (tick FOC) ne sont jamais réalloués.
// ============================================================================
#pragma once
#include <Arduino.h>

namespace brake {

// Configure TIM2 en center-aligned et laisse le demi-pont à l'ARRÊT.
// À appeler dans setup() APRÈS la mise en route du DRV8301.
void init();

// Régulation du chopper. À appeler périodiquement depuis SafetyTask.
//   vbus         : tension de bus filtrée (V). <= 0 => mesure invalide => OFF.
//   stage_active : étage de puissance réellement actif (g_focReady && !g_fault
//                  && EN_GATE haut). Faux => OFF immédiat, sans condition.
void update(float vbus, bool stage_active);

// Arrêt immédiat des deux FETs. Sûr à appeler avant init() et depuis
// n'importe quel contexte (deux écritures registre).
void off();

// Duty actuellement appliqué [0..CFG_BRAKE_MAX_DUTY], pour la télémétrie.
float duty();

} // namespace brake
