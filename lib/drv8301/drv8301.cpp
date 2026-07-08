// ============================================================================
//  drv8301.cpp
// ============================================================================
#include "drv8301.h"

// DRV8301 SPI: up to 10 MHz, MSB first, mode 1 (CPOL=0, CPHA=1). 1 MHz is plenty.
static const SPISettings DRV_SPI_CFG(1000000, MSBFIRST, SPI_MODE1);

void DRV8301::begin() {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);      // idle high (active low)
  _spi.begin();
}

uint16_t DRV8301::transfer16(uint16_t w) {
  _spi.beginTransaction(DRV_SPI_CFG);
  digitalWrite(_cs, LOW);
  uint16_t r = _spi.transfer16(w);
  digitalWrite(_cs, HIGH);
  _spi.endTransaction();
  return r;
}

void DRV8301::writeReg(uint8_t addr, uint16_t data) {
  // RW=0, addr in bits 14..11, data in bits 10..0
  uint16_t w = ((uint16_t)(addr & 0x0F) << 11) | (data & 0x07FF);
  transfer16(w);
}

uint16_t DRV8301::readReg(uint8_t addr) {
  // RW=1; the response to a read arrives on the following frame (pipelined).
  uint16_t w = 0x8000 | ((uint16_t)(addr & 0x0F) << 11);
  transfer16(w);                 // issue the read request
  uint16_t r = transfer16(w);    // r = data for the request above
  return r & 0x07FF;
}

bool DRV8301::setGain(Gain g) {
  // CONTROL2: GAIN occupies bits 3:2 (00=10, 01=20, 10=40, 11=80 V/V).
  writeReg(REG_CONTROL2, (uint16_t)g << 2);
  uint16_t rb = readReg(REG_CONTROL2);
  return (((rb >> 2) & 0x03) == (uint16_t)g);
}
