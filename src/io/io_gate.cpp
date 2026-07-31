// ============================================================================
//  io_gate.cpp — see io_gate.h.
// ============================================================================
#include "io/io_gate.h"
#include <SPI.h>
#include "config/hw_pinout.h"
#include "config/motor_config.h"

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
