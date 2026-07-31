// ============================================================================
//  prog_console.h — PROGRAM PRG_CONSOLE: the USB serial debug console. Runs at
//  10 Hz (the slowest task): it executes any commands that arrived and emits
//  one telemetry line per scan.
// ============================================================================
#pragma once
#include "plc/plc_program.h"

namespace prog {

class PrgConsole : public plc::Program {
 public:
  const char* name() const override { return "PRG_CONSOLE"; }
  void scan() override;

 private:
  uint32_t _beat = 0;
};

extern PrgConsole prgConsole;

// Prints the command help, generated from include/console_commands.h. Called
// by the CONFIGURATION at the end of boot, before the scheduler starts.
void printConsoleBanner();

} // namespace prog
