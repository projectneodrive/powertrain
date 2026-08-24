// ============================================================================
//  bridge_telemetry.cpp — see bridge_telemetry.h.
// ============================================================================
#include "bridge_telemetry.h"

#include "bridge_axis.h"   // Axis::kLinkTimeoutMs / kHeartbeatPeriodMs
#include "log.h"

namespace bridge {
namespace channel {

// ---------------------------------------------------------------------------
//  Channels the bridge CAN source, from the frames the board broadcasts.
//  The have_* guards matter: before the first Get_Bus_VI arrives, "0.0 V" is a
//  reading nobody took, and on a plot it is indistinguishable from a dead bus.
// ---------------------------------------------------------------------------

// The setpoint WE sent, in the unit of the active mode — not a measurement.
bool tgt(const State& s, float& out) { out = s.c.target; return true; }

bool Iq(const State& s, float& out) {
  if (!s.m.have_iq) return false;
  out = s.m.iq_measured_a;
  return true;
}

bool vel(const State& s, float& out) {
  if (!s.m.have_encoder) return false;
  out = s.m.vel_rad_s;
  return true;
}

bool pos(const State& s, float& out) {
  if (!s.m.have_encoder) return false;
  out = s.m.pos_rad;
  return true;
}

bool Vbus(const State& s, float& out) {
  if (!s.m.have_vbus) return false;
  out = s.m.vbus_v;
  return true;
}

// Regenerated current: ibus < 0 means the motor is pushing back into the bus.
// Plotted positive so it compares directly against Ibrk, exactly as the
// firmware's own expression does.
bool Irgn(const State& s, float& out) {
  if (!s.m.have_vbus) return false;   // ibus arrives in the same frame as vbus
  out = (s.m.ibus_a < 0.0f) ? -s.m.ibus_a : 0.0f;
  return true;
}

// ---------------------------------------------------------------------------
//  Channels the bridge CANNOT source. Not an oversight — there is no CANSimple
//  command that carries them, so they exist only on the board's own USB stream.
// ---------------------------------------------------------------------------

// Brake-resistor current is duty * Vbus / R, and the brake chopper's duty cycle
// is not published on CAN at all. Reporting 0 would read as "the brake never
// fires", which is the opposite of a diagnosis.
bool Ibrk(const State&, float&) { return false; }

// Sensorless/hall blend fraction and the observer's disagreement with the hall:
// internals of the observer, likewise not on the wire. Both matter when the
// handoff misbehaves, so this is a real reason to plug into the board's own USB
// port rather than driving through the station.
bool blnd(const State&, float&) { return false; }
bool obsdV(const State&, float&) { return false; }

}  // namespace channel

// ---------------------------------------------------------------------------
bool emitTelemetry(uint32_t now_ms, State& s, const cansimple::Link& link,
                   bool link_fresh) {
  static uint32_t last = 0;
  if (now_ms - last < BRIDGE_TELEMETRY_MS) {
    return false;
  }
  last = now_ms;

  Serial.print("t=");     Serial.print(now_ms);
  Serial.print(" #");     Serial.print(s.sample_index++);
  Serial.print(" mode="); Serial.print(s.c.control_mode);

  // The channels, from the schema shared with the firmware and the web GUI.
  // Both macros expand the same way here: whether a channel is hall-only is a
  // firmware build question, and over CAN we simply never receive it.
#define TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr)  \
  {                                                               \
    float v_ = 0.0f;                                              \
    if (channel::key(s, v_)) {                                    \
      Serial.print(" " #key "=");                                 \
      Serial.print(v_, prec);                                     \
    }                                                             \
  }
#define TELEMETRY_CHANNEL_HALL TELEMETRY_CHANNEL
#include "telemetry_schema.h"
#undef TELEMETRY_CHANNEL_HALL
#undef TELEMETRY_CHANNEL

  // Status word, in the firmware's vocabulary so the GUI's parsing is unchanged.
  // The board derives it from its own foc_ready/calibrated flags, which are not
  // on the wire; the heartbeat's axis state is the closest CAN equivalent.
  const bool running = link_fresh &&
                       s.m.heartbeat_state == odcan::AXIS_CLOSED_LOOP &&
                       s.m.axis_error == 0;
  Serial.print(running ? " RUN" : (link_fresh ? " idle" : " SAFE"));

  if (s.m.axis_error != 0) {
    Serial.print(" [FAULT]");
    Serial.print(" err=0x"); Serial.print(s.m.axis_error, HEX);
  }

  // Same counter names as the firmware's line, but these are the BRIDGE's own
  // TX/RX totals — i.e. this end of the link, which is the end you cannot see
  // from the board.
  Serial.print(" can_tx_ok=");   Serial.print(link.txOkCount());
  Serial.print(" can_tx_fail="); Serial.print(link.txFailCount());
  Serial.print(" can_rx=");      Serial.println(link.rxCount());
  return true;
}

// ---------------------------------------------------------------------------
bool emitCanStatus(uint32_t now_ms, State& s, Axis& axis,
                   const cansimple::Link& link, candiag::Diagnostics& diag) {
  static uint32_t last = 0;
  if (now_ms - last < BRIDGE_CAN_STATUS_MS) {
    return false;
  }
  last = now_ms;

  candiag::BusStatus bus;
  if (!diag.readBus(bus)) {
    return false;
  }

  // Heartbeat age is what makes "link=0" actionable: 600 ms means it just
  // dropped, 90000 ms means it was never there.
  const uint32_t hb_age = axis.heartbeatAge(now_ms);
  const uint64_t nodes  = diag.seenNodes();

  Serial.printf(
      "can node=%u link=%u hb_age=%lu hb_period=%lu hb_timeout=%lu hb_max=%lu "
      "drops=%lu scan_max=%lu stop_after=%lu stopped=%u bus=%d "
      "axis=%u mode=%u axis_err=0x%lX motor_err=0x%lX enc_err=0x%lX ctrl_err=0x%lX "
      "tx_ok=%lu tx_fail=%lu rx=%lu tx_ec=%lu rx_ec=%lu tx_failed=%lu rx_missed=%lu "
      "rx_overrun=%lu arb_lost=%lu bus_ec=%lu baud=%lu nodes=0x%08lX%08lX loglvl=%u\n",
      (unsigned)link.targetNode(),
      (unsigned)(s.link_up ? 1 : 0),
      (unsigned long)hb_age,
      (unsigned long)Axis::kHeartbeatPeriodMs,
      (unsigned long)Axis::kLinkTimeoutMs,
      (unsigned long)axis.takeMaxHeartbeatGap(),
      (unsigned long)axis.linkDropCount(),
      (unsigned long)s.scan_max_ms,
      (unsigned long)Axis::kLinkLossStopMs,
      (unsigned)(s.safety_stopped ? 1 : 0),
      bus.state,
      (unsigned)s.m.heartbeat_state,
      (unsigned)s.c.control_mode,
      (unsigned long)s.m.axis_error,
      (unsigned long)s.m.motor_error,
      (unsigned long)s.m.encoder_error,
      (unsigned long)s.m.controller_error,
      (unsigned long)link.txOkCount(),
      (unsigned long)link.txFailCount(),
      (unsigned long)link.rxCount(),
      (unsigned long)bus.tx_ec,
      (unsigned long)bus.rx_ec,
      (unsigned long)bus.tx_failed,
      (unsigned long)bus.rx_missed,
      (unsigned long)bus.rx_overrun,
      (unsigned long)bus.arb_lost,
      (unsigned long)bus.bus_ec,
      (unsigned long)CFG_CAN_BAUD,
      (unsigned long)(nodes >> 32), (unsigned long)(nodes & 0xFFFFFFFFul),
      (unsigned)logx::level());

  s.scan_max_ms = 0;   // high-water mark consumed; next line reports the next second
  return true;
}

}  // namespace bridge
