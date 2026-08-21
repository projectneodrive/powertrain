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
  //
  // `now_ms` is THE SCAN'S clock, passed in rather than read here. Reading
  // millis() inside the decoders is what broke this once already: poll() samples
  // the time, then draining and tracing the queue takes milliseconds, so a
  // heartbeat stamped with its own millis() landed AFTER the scan's `now`.
  // The next line computed `now - last_heartbeat_ms`, underflowed, and read as
  // 4.29 billion ms of silence — so every timeout fired at once and the station
  // disarmed the motor while heartbeats were streaming in.
  void onFrame(const cansimple::Frame& frame, uint32_t now_ms);

  // Milliseconds since the last heartbeat, clamped at zero.
  //
  // The clamp is not defensive noise: an age is a duration and durations are
  // not negative, but the natural expression for one is an unsigned subtraction
  // that silently produces 4.29 billion when the operands are out of order.
  // Every timeout in this class is a comparison against this value.
  uint32_t heartbeatAge(uint32_t now_ms) const;

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

  // ---- why the link keeps dropping ----------------------------------------
  // The bus counters say whether frames are being CORRUPTED (bus_ec/rx_ec) or
  // DROPPED by this end (rx_missed/rx_overrun). Neither says anything when the
  // heartbeats simply arrive late — a board whose COMMS task is being starved
  // looks exactly like a board that stopped, and both look like a clean bus.
  //
  // So: the worst gap actually observed between two heartbeats. Compare it
  // against kHeartbeatPeriodMs. Steady 100 ms then a long silence means the
  // sender stopped; a spread of 150-400 ms means frames are being lost or
  // delayed, and kLinkTimeoutMs is simply tighter than the link deserves.
  //
  // Two accumulators, because they have two readers with different questions.
  // The status line CONSUMES the windowed one, so it reads "worst in the last
  // second" — live, and not a high-water mark from a glitch at boot. The
  // link-lost message needs the worst gap over the whole connection, and must
  // not be left reading whatever the status line happened to reset it to.
  uint32_t takeMaxHeartbeatGap();          // windowed; resets on read
  uint32_t worstHeartbeatGap() const { return _hb_max_session; }

  // Times the link has been declared lost since boot. A drift check for the
  // above: without it, "very often" is a feeling rather than a rate.
  uint32_t linkDropCount() const { return _link_drops; }

  // The station was blocked for `stall_ms` and saw nothing during it. Credits
  // that time back to the heartbeat clock, so a stall here cannot be mistaken
  // for silence out there. Called by ControlStation::poll(); see
  // BRIDGE_SCAN_STALL_MS for why this exists.
  void creditStall(uint32_t stall_ms, uint32_t now_ms);

  // ---- link-loss safety stop -----------------------------------------------
  // How long out of contact before the axis is disarmed. Much longer than the
  // link-lost timeout on purpose: losing the link is worth reporting at once, a
  // brief dropout is not worth stopping a motor for. 0 disables it.
  static constexpr uint32_t kLinkLossStopMs = BRIDGE_LINK_LOSS_STOP_MS;
  static_assert(kLinkLossStopMs == 0 || kLinkLossStopMs >= kLinkTimeoutMs,
                "BRIDGE_LINK_LOSS_STOP_MS is shorter than the link-lost timeout: "
                "the axis would be disarmed before the link is even declared down.");

  // Disarm the axis if the heartbeat has been gone for kLinkLossStopMs, ONCE.
  //
  // Once, because repeating it every scan would overwrite whatever the operator
  // types next — and when the bus is the broken thing, the console has to stay
  // usable. It re-arms the one-shot when a heartbeat comes back, so a link that
  // drops twice stops the motor twice.
  //
  // Only fires after a heartbeat has been seen at least once: with no board
  // ever present there is no falling edge, and nothing to stop.
  void checkLinkLossStop(uint32_t now_ms);

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
#define CAN_TX_CYCLIC(cmd, period_ms, sender) \
  void rx_##sender(const uint8_t* b, uint8_t len, uint32_t now_ms);
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

  bool _ever_linked = false;   // a heartbeat has been received at least once
  bool _stop_sent   = false;   // the one-shot safety stop has fired

  // A disarm we sent should be reflected in the board's heartbeat within a few
  // heartbeat periods. Longer than that and crediting the change to our command
  // would be a guess; the point of the attribution is to be trustworthy.
  static constexpr uint32_t kDisarmAttributionMs = kHeartbeatPeriodMs * 5;
  uint32_t _last_disarm_cmd_ms = 0;

  uint32_t _hb_prev_ms     = 0;   // arrival time of the previous heartbeat
  uint32_t _hb_max_window  = 0;   // worst gap since the last status line
  uint32_t _hb_max_session = 0;   // worst gap since the link came up
  uint32_t _link_drops     = 0;   // link-lost transitions since boot
  // False across an outage: the silence between the last heartbeat before a
  // drop and the first one after is NOT a gap in the heartbeat stream, it IS
  // the outage. Counting it would report every reconnection as the worst
  // raggedness ever seen, which is the opposite of a useful measurement.
  bool     _hb_gap_valid   = false;
  // Stall time forgiven since the last heartbeat, capped at kLinkLossStopMs so
  // a station that stalls repeatedly cannot postpone the safety stop forever.
  // Reset by every heartbeat — being in contact is what earns the allowance back.
  uint32_t _stall_credit_ms = 0;

  cansimple::Link& _link;
  State&           _st;
};

}  // namespace bridge
