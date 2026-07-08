// ============================================================================
//  drv8301.h  —  minimal SPI driver for the TI DRV8301 gate driver.
//
//  SimpleFOC has no DRV8301 driver, so we configure it by hand. The only thing
//  we strictly need is to set the current-shunt amplifier GAIN so it matches the
//  value passed to SimpleFOC's LowsideCurrentSense; we also expose the fault
//  status registers.
//
//  Wiring (ODrive v3.6): SPI3 — SCK PC10, MISO PC11, MOSI PC12, CS PC13.
//  The DRV must be awake (EN_GATE high) for SPI access; call setGain() again
//  after each enable in case EN_GATE was toggled low (safe state).
//
//  SPI frame (16-bit, MSB first, mode 1):
//     bit15 = R/W (1=read, 0=write) · bits14..11 = address · bits10..0 = data
//  Reads are pipelined: the data for a read command comes back on the *next*
//  frame, so readReg() issues two transfers.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>

class DRV8301 {
public:
  enum Gain : uint8_t { GAIN_10 = 0, GAIN_20 = 1, GAIN_40 = 2, GAIN_80 = 3 };

  // DRV8301 registers
  static const uint8_t REG_STATUS1 = 0x00;
  static const uint8_t REG_STATUS2 = 0x01;
  static const uint8_t REG_CONTROL1 = 0x02;
  static const uint8_t REG_CONTROL2 = 0x03;

  DRV8301(SPIClass& spi, int cs_pin) : _spi(spi), _cs(cs_pin) {}

  void     begin();                       // SPI + CS pin
  bool     setGain(Gain g);               // write CTRL2 gain, verify by readback
  uint16_t readReg(uint8_t addr);         // 11-bit register value
  void     writeReg(uint8_t addr, uint16_t data);
  uint16_t status1() { return readReg(REG_STATUS1); }
  uint16_t status2() { return readReg(REG_STATUS2); }
  bool     faulted() { return (status1() & (1u << 10)) != 0; }  // bit10 = aggregate FAULT

  // Map a numeric gain (V/V) to the register code.
  static Gain gainFromVpV(float vpv) {
    if (vpv >= 80.0f) return GAIN_80;
    if (vpv >= 40.0f) return GAIN_40;
    if (vpv >= 20.0f) return GAIN_20;
    return GAIN_10;
  }

private:
  uint16_t transfer16(uint16_t w);
  SPIClass& _spi;
  int       _cs;
};
