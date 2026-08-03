// ============================================================================
//  bridge_axis.h — the axis, as reachable over CANSimple.
//
//  One method per thing an operator can ask for, and one decoder per frame the
//  board broadcasts. Everything above this (the console, the joystick) speaks
//  in SI units and axis verbs; everything below it (lib/cansimple) speaks in
//  frames. This is the only file that knows the wire encoding of a payload.
//
//  UNITS. Firmware-side and console-side everything is SI/rad. CANSimple is in
//  revolutions, so every conversion happens here, at the boundary — mirroring
//  lib/odrive_can/odrive_can.cpp on the board, which converts back. If you
//  change one you must change the other; they are two halves of one encoding.
//
//  THE RX SIDE IS GENERATED. The decoders are declared from the firmware's
//  CAN_TX_CYCLIC list, so adding a cyclic telemetry frame on the board makes
//  THIS build fail to link until a decoder for it exists. A silently ignored
//  new telemetry channel is exactly the failure that table is meant to prevent.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "axis_names.h"
#include "bridge_state.h"
#include "cansimple.h"
#include "log.h"

namespace bridge {

class Axis {
 public:
  Axis(cansimple::Link& link, State& state) : _link(link), _st(state) {}

  // ---- state ---------------------------------------------------------------
  // Arm: clear any latched error, put the axis in a known mode at a zero
  // setpoint, THEN close the loop. Order matters — closing the loop first would
  // arm onto whatever setpoint was left over from the previous session.
  bool arm();
  bool idle();
  bool estop();
  bool clearErrors();
  bool reboot();
  bool characterise();          // measure phase R/L (AXIS_MOTOR_CAL)

  // ---- setpoints -----------------------------------------------------------
  bool setControllerMode(uint8_t mode, uint8_t input_mode);
  bool setTorque(float nm);
  bool setVelocity(float rad_s);
  bool setPosition(float rad);

  // Send a velocity without re-asserting the mode when already in it. The
  // joystick calls this at 10 Hz; re-asserting would double the bus traffic on
  // every tick, and would also stop a one-off T/X command from ever holding.
  bool driveVelocity(float rad_s);

  // ---- configuration -------------------------------------------------------
  // Both of these send the whole tuple from State::c, because CANSimple packs
  // two values per frame and there is no way to set one alone.
  bool applyLimits();           // Set_Limits(vel_limit, current_limit)
  bool applyVelGains();         // Set_Vel_Gains(P, I)  — no D term on the wire
  bool applyPosGain();          // Set_Pos_Gain(P)

  // ---- diagnostics ---------------------------------------------------------
  bool requestErrors();         // ask for motor / encoder / controller errors

  // ---- receive -------------------------------------------------------------
  // Decode one frame. Frames from other nodes are dropped (the caller has
  // already traced them), so a second board on the bus cannot move this axis.
  void onFrame(const cansimple::Frame& frame);

  // Heartbeat freshness. The timeout is derived from the firmware's own
  // broadcast period rather than written down here, so retuning the heartbeat
  // rate on the board retunes this with it.
  static constexpr uint32_t kHeartbeatPeriodMs =
      cansimple::cyclicPeriodMs(odcan::CMD_HEARTBEAT);
  static constexpr uint32_t kLinkTimeoutMs =
      kHeartbeatPeriodMs * BRIDGE_HEARTBEAT_MISSES;
  static_assert(kHeartbeatPeriodMs > 0,
                "The firmware no longer broadcasts CMD_HEARTBEAT cyclically "
                "(include/can_commands.h) — the link-loss timeout has nothing "
                "to be based on. Pick another cyclic frame to watch.");

  // Update State::link_up and report it, printing the transition once.
  bool refreshLink(uint32_t now_ms);

 private:
  bool setAxisState(uint8_t state);

  // Reply-only frames: the firmware answers these on request but never
  // broadcasts them, so they are not in the CAN_TX_CYCLIC list and are handled
  // by hand. Kept small on purpose.
  void onReply(const cansimple::Frame& frame);

  // One decoder per cyclic frame the firmware broadcasts, named after the
  // sender that emits it on the board. Declared from the shared table; defined
  // in bridge_axis.cpp. A missing one is a link error, by design.
#define CAN_RX(cmd, handler)
#define CAN_TX_CYCLIC(cmd, period_ms, sender)  void rx_##sender(const uint8_t* b, uint8_t len);
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX

  // Edge detectors. The heartbeat arrives at 10 Hz and carries the same two
  // values over and over; these are what turn that stream into the two lines
  // an operator actually wants — "state IDLE -> CLOSED_LOOP" and a decoded
  // error word the moment a bit appears.
  logx::OnChange<uint8_t>  _state_changed;
  logx::OnChange<uint32_t> _error_changed;
  logx::OnChange<bool>     _link_changed;

  cansimple::Link& _link;
  State&           _st;
};

}  // namespace bridge
