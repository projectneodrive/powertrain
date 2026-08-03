// ============================================================================
//  can_diag.cpp — see can_diag.h.
// ============================================================================
#include "can_diag.h"

namespace candiag {

Diagnostics* Diagnostics::_self = nullptr;

const char* busStateName(int state) {
  switch (state) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default:                    return "?";
  }
}

void Diagnostics::tap(const char* dir, const cansimple::Frame& frame, bool ok) {
  if (_self == nullptr) return;
  _self->noteNodeSeen(frame.node);

  if (!ok) {
    // A rejected transmit is not trace, it is a failure: the command the
    // operator just issued did not leave the building. Always reported.
    LOG_E("CAN", "TX REJECTED id=0x%03lX %s - controller would not accept the frame",
          (unsigned long)frame.id, cansimple::cmdName(frame.cmd));
    return;
  }
  _self->logFrame(dir, frame);
}

// The per-frame trace. DEBUG, so it is off unless asked for with `D3` — at
// 100 Hz of cyclic telemetry it is the single largest source of noise, and it
// is only ever wanted when you are looking at one specific exchange.
void Diagnostics::logFrame(const char* dir, const cansimple::Frame& frame) {
  if (logx::level() < logx::LVL_DEBUG) return;

  char payload[32] = {0};
  size_t used = 0;
  for (uint8_t i = 0; i < frame.len && used + 3 < sizeof(payload); i++) {
    used += snprintf(payload + used, sizeof(payload) - used, "%02X", frame.data[i]);
  }

  LOG_D("CAN", "%s id=0x%03lX node=%u %s len=%u %s", dir, (unsigned long)frame.id,
        (unsigned)frame.node, cansimple::cmdName(frame.cmd), (unsigned)frame.len, payload);
}

void Diagnostics::noteNodeSeen(uint8_t node) {
  const uint64_t bit = (uint64_t)1 << (node & 0x3F);
  if ((_seen_nodes & bit) != 0) return;
  _seen_nodes |= bit;

  if (node == _target) {
    LOG_I("CAN", "node %u seen on bus (target)", (unsigned)node);
  } else {
    LOG_W("CAN", "node %u seen on bus, NOT the target (%u) - check CFG_CAN_NODE_ID "
                 "in include/config/motor_config.h",
          (unsigned)node, (unsigned)_target);
  }
}

void Diagnostics::pollAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK || alerts == 0) return;

  // These fire per event. On a marginal link that can mean thousands per
  // second — which is why they go through the logger's folding rather than
  // being filtered out here: "repeated 4213 times" is the diagnosis.
  if (alerts & TWAI_ALERT_BUS_ERROR) {
    LOG_W("BUS", "bus error (bit/stuff/crc/form/ack) - check wiring, termination, bit rate");
  }
  if (alerts & TWAI_ALERT_ERR_PASS) {
    LOG_W("BUS", "controller ERROR-PASSIVE - link unreliable, check termination/wiring");
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    LOG_W("BUS", "RX queue full, frames dropped (bus flooded or the loop is not draining)");
  }
  if (alerts & TWAI_ALERT_BUS_OFF) {
    LOG_E("BUS", "BUS-OFF: too many errors, controller disabled itself - recovering");
    twai_initiate_recovery();
  }
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    LOG_I("BUS", "recovered from BUS-OFF, restarting driver");
    twai_start();
  }
}

bool Diagnostics::readBus(BusStatus& out) {
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return false;

  out.state      = s.state;
  out.tx_ec      = s.tx_error_counter;
  out.rx_ec      = s.rx_error_counter;
  out.tx_failed  = s.tx_failed_count;
  out.rx_missed  = s.rx_missed_count;
  out.rx_overrun = s.rx_overrun_count;
  out.arb_lost   = s.arb_lost_count;
  out.bus_ec     = s.bus_error_count;

  // Skip the very first observation: coming up RUNNING is not news, and
  // cansimple::Link::begin() already said so.
  if (_bus_state(out.state) && !_bus_state.first()) {
    LOG_W("BUS", "controller state %s -> %s", busStateName(_bus_state.previous()),
          busStateName(out.state));
  }

  // The counters climbing while the state stays RUNNING is a link that is
  // dropping frames but has not failed — the classic missing-terminator
  // signature. Reported once on the transition, not every sample; the live
  // numbers are on the CAN Devices page.
  const bool counting = (out.bus_ec != 0) || (out.tx_ec != 0) || (out.rx_ec != 0);
  if (_errors_counting(counting) && !(_errors_counting.first() && !counting)) {
    if (counting) {
      LOG_W("BUS", "error counters climbing while state=%s (tx_ec=%lu rx_ec=%lu bus_ec=%lu) "
                   "- marginal link, suspect termination",
            busStateName(out.state), (unsigned long)out.tx_ec, (unsigned long)out.rx_ec,
            (unsigned long)out.bus_ec);
    } else {
      LOG_I("BUS", "error counters back to zero");
    }
  }
  return true;
}

}  // namespace candiag
