// ============================================================================
//  can_bridge.cpp — see can_bridge.h.
// ============================================================================
#include "can_bridge.h"

#include "log.h"

namespace bridge {

ControlStation station;

void ControlStation::begin() {
  Serial.begin(BRIDGE_SERIAL_BAUD);
  delay(200);

  _link.begin();

  // The tracer is a plain function pointer, so it needs to know which instance
  // to forward to. One station drives one bus; a second would be a bug.
  candiag::Diagnostics::bind(&_diag);
  _link.onFrame(&candiag::Diagnostics::tap);

  // Before anything is armed: the pot is at rest right now, which is the only
  // moment its zero can be measured without asking anybody to hold it there.
  _pot.begin();

  console::Context ctx;
  ctx.axis  = &_axis;
  ctx.state = &_state;
  ctx.pot   = &_pot;
  console::begin(ctx);

  LOG_I("SYS", "CAN control station ready - target node %u @ %lu kbit/s "
               "(from include/config/motor_config.h)",
        (unsigned)BRIDGE_TARGET_NODE_ID, (unsigned long)(CFG_CAN_BAUD / 1000));
  LOG_I("SYS", "log level %s - D0..D3 to change, D3 adds the per-frame CAN trace",
        logx::levelName(logx::level()));
  console::printBanner();

  // Put the axis somewhere known. The limits sent here are the FIRMWARE's own
  // CFG_VEL_LIMIT / CFG_CURRENT_LIMIT, so this is a confirmation rather than an
  // override — which is the point: the station cannot arm the motor against a
  // limit the board was not configured for. The gains are deliberately NOT
  // pushed; the board already booted with them, and resending would stamp on a
  // live tuning session running over its USB console.
  _axis.clearErrors();
  _axis.applyLimits();
  _axis.setControllerMode(odcan::CTRL_VELOCITY, INPUT_PASSTHROUGH);
  _axis.setVelocity(0.0f);
}

void ControlStation::poll() {
  const uint32_t now = millis();

  // 0. Was this station blocked since the last scan?
  //
  //    A scan takes microseconds. A long one means we were stuck somewhere —
  //    almost always a serial write waiting on a host that stopped reading —
  //    and during it the TWAI receive queue was not being drained, so frames
  //    were queued and, past BRIDGE_RX_QUEUE_LEN, dropped by the driver.
  //
  //    Reporting this is the point. Without it the stall is invisible and
  //    presents as the board misbehaving: the heartbeat age jumps past every
  //    timeout at once, and the station declares a link loss and disarms the
  //    motor in the same scan, for a board that never stopped transmitting.
  //    CREDITING and WARNING are two different thresholds, and conflating them
  //    was a bug. The link timeout is only five heartbeat periods, so a scan
  //    that takes even a fraction of one makes the check sample too coarsely:
  //    at D3 the frame trace slowed scans to ~100-250 ms, under the warning
  //    threshold, and the link flapped and eventually disarmed the motor for no
  //    reason a human could see. So: credit silently from one heartbeat period
  //    (correctness), warn only past BRIDGE_SCAN_STALL_MS (a human's business).
  if (_last_scan_ms != 0) {
    const uint32_t scan_gap = now - _last_scan_ms;
    if (scan_gap > _state.scan_max_ms) _state.scan_max_ms = scan_gap;
    if (scan_gap > Axis::kHeartbeatPeriodMs) {
      if (scan_gap > BRIDGE_SCAN_STALL_MS) {
        LOG_W("SYS", "loop stalled %lums - CAN frames were not being drained "
                     "(frame trace at D3? serial host not reading?). "
                     "Link timers credited back.",
              (unsigned long)scan_gap);
      }
      _axis.creditStall(scan_gap, now);
    }
  }
  _last_scan_ms = now;

  // 1. Read inputs — drain the whole RX queue, not one frame per scan, or a
  //    burst outruns the loop and the queue overflows (see BRIDGE_RX_QUEUE_LEN).
  //    `now` is passed in, not re-read per frame: draining and tracing the
  //    queue takes real milliseconds, and a heartbeat stamped with a later
  //    clock than the one the timeouts below are checked against underflowed
  //    into 4.29 billion ms of "silence".
  cansimple::Frame frame;
  while (_link.receive(frame)) {
    _axis.onFrame(frame, now);
  }

  // 2. Operator input.
  console::poll();

  float pot_vel = 0.0f;
  if (_pot.poll(now, pot_vel)) {
    if (_axis.driveVelocity(pot_vel)) {
      // DEBUG, not an acknowledgement: the joystick moves continuously and its
      // value is already visible as `tgt` on every telemetry line. Logging it
      // was ten lines a second saying what the plot was already showing.
      LOG_D("POT", "adc=%d vel=%.3f rad/s", _pot.raw(), (double)pot_vel);
    } else {
      LOG_E("POT", "velocity command failed to transmit");
    }
  }

  // 3. Safety, then report. refreshLink() before the telemetry line so a link
  //    that just dropped is reported as SAFE on this line and not the next one;
  //    the safety stop between them so its effect is on the same line too.
  const bool link_fresh = _axis.refreshLink(now);
  _axis.checkLinkLossStop(now);
  emitTelemetry(now, _state, _link, link_fresh);

  // 4. Bus health. Alerts are events (they go to the log); the counters are
  //    state and go out on the `can ...` line, which the GUI turns into a page.
  _diag.pollAlerts();
  emitCanStatus(now, _state, _axis, _link, _diag);

  // 5. Close out any folded "repeated N times" summary whose window has
  //    expired. Last, so it never splits a burst it is summarising.
  logx::tick(now);
}

}  // namespace bridge
