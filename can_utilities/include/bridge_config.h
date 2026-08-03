// ============================================================================
//  bridge_config.h — the ESP32 control station's configuration.
//
//  Two kinds of setting live here, and the distinction is the whole point:
//
//   1. WHAT IS WIRED TO THIS ESP32 — transceiver pins, the potentiometer, the
//      serial rates. Nobody else can know these, so they are defined below.
//
//   2. WHAT MUST MATCH THE BOARD — CAN node id, bit rate, limits, gain
//      defaults. These are NOT defined here. They are #included from the
//      firmware's own config/motor_config.h, one directory up, so that there is
//      exactly one place to change them and no way for the two ends of the bus
//      to disagree. Editing the board's config re-flashes this bridge with the
//      new values on its next build.
//
//  Overriding any CFG_* value here would defeat that, so don't: change it in
//  ../include/config/motor_config.h.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "driver/twai.h"

// The board's configuration — the source of truth for everything shared.
// Also defines SENSOR_TYPE, which console_commands.h tests when it decides
// whether the hall-calibration command exists.
#include "config/motor_config.h"

// ---------------------------------------------------------------------------
//  CAN: node id and bit rate come from the firmware config
// ---------------------------------------------------------------------------
// The node this station drives. CFG_CAN_NODE_ID is the board's own id, so the
// default is "the board this repo builds". Override on the command line
// (-D BRIDGE_TARGET_NODE_ID=3) to drive a differently-configured board without
// touching either config file.
#ifndef BRIDGE_TARGET_NODE_ID
#define BRIDGE_TARGET_NODE_ID  CFG_CAN_NODE_ID
#endif

// CFG_CAN_BAUD is a plain number; the TWAI driver wants one of its own timing
// constants. Translating here is what makes a bit-rate change on the firmware
// side either work or refuse to compile — never silently produce a bridge
// talking at the wrong rate, which on CAN looks like a wiring fault.
#if   CFG_CAN_BAUD == 1000000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_1MBITS()
#elif CFG_CAN_BAUD == 800000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_800KBITS()
#elif CFG_CAN_BAUD == 500000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_500KBITS()
#elif CFG_CAN_BAUD == 250000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_250KBITS()
#elif CFG_CAN_BAUD == 125000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_125KBITS()
#elif CFG_CAN_BAUD == 100000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_100KBITS()
#elif CFG_CAN_BAUD == 50000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_50KBITS()
#elif CFG_CAN_BAUD == 25000
  #define BRIDGE_TWAI_TIMING_CONFIG()  TWAI_TIMING_CONFIG_25KBITS()
#else
  #error "CFG_CAN_BAUD (../include/config/motor_config.h) has no ESP32 TWAI timing constant. Pick a supported rate, or add the case above."
#endif

// ---------------------------------------------------------------------------
//  ESP32 wiring — local to this station
// ---------------------------------------------------------------------------
// TWAI controller pins. Adjust to your board and 3.3 V transceiver (CJMCU-230
// or similar). These are ESP32 GPIO numbers, not the transceiver's pin numbers.
#define BRIDGE_TWAI_TX_PIN   GPIO_NUM_5
#define BRIDGE_TWAI_RX_PIN   GPIO_NUM_4

// Default rx_queue_len is only 5. The board's cyclic timers (heartbeat,
// encoder, Iq, Vbus — see the CAN_TX_CYCLIC list in can_commands.h) all fire
// together on their first crossing, so it can burst 4+ frames in the same
// millisecond right at boot, while setup() is still blocking on Serial/TX and
// has not started draining the queue.
#define BRIDGE_RX_QUEUE_LEN  32
#define BRIDGE_TX_QUEUE_LEN  16

// How long twai_transmit() may block waiting for a free TX mailbox.
#define BRIDGE_TX_TIMEOUT_MS 10

// ---------------------------------------------------------------------------
//  Host serial link (to GUI/serial_plotter_wasm or serial_plotter_fast.py)
// ---------------------------------------------------------------------------
#define BRIDGE_SERIAL_BAUD      115200
// Telemetry line period. Matches the board's own SER task (100 ms) so the GUI
// sees the same sample rate whichever port it is plugged into.
#define BRIDGE_TELEMETRY_MS     100
// Period of the machine-readable `can ...` status line — node id, link state,
// error words, bus counters. This is DATA, not logging: the GUI's CAN Devices
// page consumes it and it never reaches the monitor pane. 1 Hz is a comfortable
// refresh for a page somebody is watching, and it is one line.
#define BRIDGE_CAN_STATUS_MS    1000
// Link is declared lost after this many missed heartbeats. The heartbeat PERIOD
// is not written down here — it is read out of the firmware's CAN_TX_CYCLIC
// table at compile time (see cansimple::cyclicPeriodMs), so retuning the
// heartbeat rate on the board retunes this timeout too.
#define BRIDGE_HEARTBEAT_MISSES 5

// ---------------------------------------------------------------------------
//  Event log (lib/logging)
// ---------------------------------------------------------------------------
// Default ceiling: 0=ERROR 1=WARN 2=INFO 3=DEBUG. INFO means state changes and
// acknowledgements are shown while the per-frame trace is not — which is the
// difference between a monitor pane you can read and one you cannot. Raise it
// at runtime with the console's `D3`, no rebuild.
#define LOG_DEFAULT_LEVEL       2

// Longest formatted message. Frame traces are the long ones (~90 chars).
#define LOG_LINE_LEN            160
// How much of a message is compared when folding repeats. Shorter than
// LOG_LINE_LEN on purpose: two frame traces differing only in their payload
// bytes are, for a reader watching a loop spin, the same message.
#define LOG_DEDUP_TEXT_LEN      96
// A repeat within this window folds into a count instead of printing.
#define LOG_DEDUP_MS            2000
// Hard ceiling on the whole stream. ERROR lines are exempt — see log.cpp.
#define LOG_MAX_LINES_PER_S     20

// ---------------------------------------------------------------------------
//  Potentiometer velocity joystick
// ---------------------------------------------------------------------------
#define POT_PIN              GPIO_NUM_34
#define POT_POLL_MS          100
#define POT_ADC_MAX          4095   // analogRead resolution default (12-bit)
#define POT_ADC_MIN          0
// Full deflection either side of rest maps to +/- this. Clamped to the
// firmware's own accepted-velocity ceiling so the joystick can never ask for a
// setpoint the board would reject or wind its integrator up against.
#define POT_VEL_MAX_RAD_S    (CFG_VEL_LIMIT < CFG_VEL_CMD_MAX ? CFG_VEL_LIMIT : CFG_VEL_CMD_MAX)
#define POT_CHANGE_DEADBAND  10     // ignore ADC deltas smaller than this (noise)
#define POT_REST_DEADBAND_ADC 100   // +/- window around rest that snaps to 0 rad/s

// Ohm-meter reading (160-4.3k ohm travel, resting at 3.3k ohm) only gives an
// ESTIMATE of the rest ADC value — it assumes the measured resistance span maps
// linearly onto the full 0..POT_ADC_MAX swing, which depends on the pot's true
// total resistance and the exact divider wiring, neither of which is known
// here. Used only as the startup default: send 'Z' with the pot at physical
// rest to capture the real value at runtime.
#define POT_OHM_MIN          160.0f
#define POT_OHM_MAX          4300.0f
#define POT_OHM_REST         3300.0f
#define POT_ADC_REST_DEFAULT \
  ((int)((POT_OHM_REST - POT_OHM_MIN) / (POT_OHM_MAX - POT_OHM_MIN) * POT_ADC_MAX + 0.5f))
