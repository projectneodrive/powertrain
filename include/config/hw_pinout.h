// ============================================================================
//  hw_pinout.h  —  Pin map and hardware topology for the ODrive v3.6 (MKS
//  clone) board. This is the physical wiring only: nothing here depends on the
//  motor fitted or on the control tuning (see motor_config.h) nor on the PLC
//  scheduling (see plc_config.h).
//
//  Target: STM32F405RGT6 @ 168 MHz, DRV8301 gate driver, single channel (M0).
//  Pin map reconciled against the authoritative ODrive v3.6 hardware:
//    - DRV8301 SPI chip-select is PC13 (the PC4/PC5 in test_driver_on.cc is wrong)
//    - M0 encoder/hall is PB4/PB5 (+PC9) = TIM3  (PB6/PB7 in the speed tests = M1)
// ============================================================================
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Gate driver (DRV8301) — TIM1 6-PWM
// ---------------------------------------------------------------------------
#define PIN_M0_INH_A   PA8    // TIM1_CH1
#define PIN_M0_INH_B   PA9    // TIM1_CH2
#define PIN_M0_INH_C   PA10   // TIM1_CH3
#define PIN_M0_INL_A   PB13   // TIM1_CH1N
#define PIN_M0_INL_B   PB14   // TIM1_CH2N
#define PIN_M0_INL_C   PB15   // TIM1_CH3N
#define PIN_EN_GATE    PB12   // DRV8301 enable (toggle LOW->HIGH to reset)
#define PIN_N_FAULT    PD2    // DRV8301 nFAULT (LOW = fault), shared M0/M1

// ---------------------------------------------------------------------------
//  DRV8301 configuration SPI (SPI3)
// ---------------------------------------------------------------------------
#define PIN_DRV_SCK    PC10   // SPI3_SCK
#define PIN_DRV_MISO   PC11   // SPI3_MISO
#define PIN_DRV_MOSI   PC12   // SPI3_MOSI
#define PIN_M0_CS      PC13   // DRV8301 M0 chip-select (active low)
#define PIN_M1_CS      PC14   // DRV8301 M1 chip-select (unused, held high)

// ---------------------------------------------------------------------------
//  Phase current sense + Vbus.
//
//  Vbus sits on a DEDICATED ADC, never the one the shunts use. Bench history of
//  what fails otherwise (real 24 V bus):
//   - analogRead()             : STM32duino DeInits the ADC -> kills the
//                                injected conversions used by the current sense.
//   - _readRegularADCVoltage() : a regular channel ON the shunts' ADC. The
//                                injected trigger (TIM1, 20 kHz) aborts/restarts
//                                the regular conversion mid sample-and-hold, and
//                                it resumes with the injected channel's voltage
//                                still in the S/H -> ~5.4 V at standstill (amps
//                                off, 0 V) and ~30 V with the motor armed (amps
//                                at ~1.65 V). Systematic false OV fault during
//                                alignment, even with 480-cycle sampling, a
//                                median filter and a 10 ms debounce.
//  PA6 = ADC12_IN6: io_vbus picks whichever of ADC1/ADC2 the current sense did
//  NOT take, so no interaction with the injected conversions is possible.
// ---------------------------------------------------------------------------
#define PIN_M0_IB      PC0    // ADC2_IN10  (phase B shunt amp)
#define PIN_M0_IC      PC1    // ADC2_IN11  (phase C shunt amp; A reconstructed)
#define PIN_VBUS       PA6    // ADC12_IN6  (bus voltage divider)

// ---------------------------------------------------------------------------
//  Encoder / Hall (M0). Quadrature -> PB4/PB5 as TIM3 AF.
//  Hall -> PB4/PB5/PC9 as EXTI GPIO. Only one sensor active at a time.
// ---------------------------------------------------------------------------
#define PIN_ENC_A      PB4    // TIM3_CH1 / Hall A
#define PIN_ENC_B      PB5    // TIM3_CH2 / Hall B
#define PIN_ENC_Z      PC9    // encoder index / Hall C

// ---------------------------------------------------------------------------
//  AUX half-bridge (brake resistor), dedicated gate driver — TIM2.
//  ODrive v3.6 topology: the resistor sits between the midpoint and GROUND. The
//  HIGH FET pulls the midpoint to DC+ (that is the one that dissipates), the LOW
//  FET pulls it back to ground. Both are driven COMPLEMENTARY with dead time,
//  exactly like the ODrive firmware (safety_critical_apply_brake_resistor_timings):
//     LOW  FET ON from 0                     to period*(1-duty) - dt  (PWM mode 1)
//     HIGH FET ON from period*(1-duty) + dt  to period                (PWM mode 2)
//  Driving a single FET produces NOTHING, and that is measured on this bench:
//   - LOW FET alone : the midpoint is pulled to ground = 0 V across the
//     resistor, hence 0 A.
//   - HIGH FET alone: its driver is bootstrapped; the bootstrap capacitor only
//     charges while the LOW FET pulls the midpoint down. With no low-side
//     switching, VB-VS stays at zero and the HIGH FET never turns on.
//  Hence the complementary drive is mandatory — and so is a non-zero dead time,
//  on pain of a hard shoot-through of the bus.
//  /!\ HARDWARE TRAP: the LM5109B's VDD is wired to GVDD, the DRV8301's INTERNAL
//  gate regulator. GVDD only exists while EN_GATE is high. The brake can
//  therefore only switch when the power stage is awake — EN_GATE low means the
//  AUX half-bridge is unpowered and the gates will not move whatever the timer
//  registers say.
// ---------------------------------------------------------------------------
#define PIN_AUX_L      PB10   // TIM2_CH3 — low-side gate  (LI input of U7)
#define PIN_AUX_H      PB11   // TIM2_CH4 — high-side gate (HI input, dissipates)
