// ============================================================================
//  cansimple.cpp — see cansimple.h.
// ============================================================================
#include "cansimple.h"

#include "log.h"

namespace cansimple {

// An if-chain rather than a switch: CMD_GET_ENCODER_ESTIMATES, CMD_GET_IQ and
// CMD_GET_BUS_VI appear in BOTH lists of the firmware's table (they answer a
// request and also broadcast on a timer), and duplicate case labels would not
// compile. First match wins, which is the same answer either way.
const char* cmdName(uint8_t cmd) {
#define CAN_RX(c, handler)                   if (cmd == (uint8_t)odcan::c) return #c;
#define CAN_TX_CYCLIC(c, period_ms, sender)  if (cmd == (uint8_t)odcan::c) return #c;
#include "can_commands.h"
#undef CAN_TX_CYCLIC
#undef CAN_RX
  return "UNKNOWN";
}

void Link::begin() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(BRIDGE_TWAI_TX_PIN, BRIDGE_TWAI_RX_PIN, TWAI_MODE_NORMAL);
  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR |
                            TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_RECOVERED;
  g_config.rx_queue_len = BRIDGE_RX_QUEUE_LEN;   // see bridge_config.h for why
  g_config.tx_queue_len = BRIDGE_TX_QUEUE_LEN;

  twai_timing_config_t t_config = BRIDGE_TWAI_TIMING_CONFIG();   // from CFG_CAN_BAUD
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // The retry loops log every attempt, and the logger folds the repeats into a
  // count — so a station wired to nothing says so once and then quietly keeps
  // a tally, instead of producing the wall of text this used to.
  while (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    LOG_E("CAN", "TWAI driver install failed - check GPIOs, transceiver wiring, driver availability");
    delay(500);
  }
  while (twai_start() != ESP_OK) {
    LOG_E("CAN", "TWAI start failed");
    delay(500);
  }

  LOG_I("CAN", "TWAI up @ %lu kbit/s, TX=GPIO%d RX=GPIO%d, target node %u",
        (unsigned long)(CFG_CAN_BAUD / 1000), (int)BRIDGE_TWAI_TX_PIN,
        (int)BRIDGE_TWAI_RX_PIN, (unsigned)_node);
}

bool Link::sendRaw(uint8_t cmd, const uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier       = odcan::arbId(_node, cmd);
  msg.data_length_code = len;
  msg.flags            = TWAI_MSG_FLAG_NONE;
  if (data != nullptr && len > 0) {
    memcpy(msg.data, data, len);
  }

  const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(BRIDGE_TX_TIMEOUT_MS));
  const bool ok = (err == ESP_OK);
  ok ? ++_tx_ok : ++_tx_fail;

  if (_tap != nullptr) {
    Frame f;
    f.id   = msg.identifier;
    f.node = _node;
    f.cmd  = cmd;
    f.len  = len;
    if (data != nullptr && len > 0) memcpy(f.data, data, len);
    _tap(ok ? "TX" : "TX-FAIL", f, ok);
  }
  if (!ok) {
    // The tap already reported WHICH command was rejected; this adds why.
    LOG_E("CAN", "twai_transmit: %s", esp_err_to_name(err));
  }
  return ok;
}

bool Link::receive(Frame& out) {
  twai_message_t msg = {};
  if (twai_receive(&msg, 0) != ESP_OK) {
    return false;
  }
  ++_rx_count;

  out.id   = msg.identifier;
  out.node = odcan::nodeOf(msg.identifier);
  out.cmd  = odcan::cmdOf(msg.identifier);
  out.len  = msg.data_length_code > 8 ? 8 : msg.data_length_code;
  memcpy(out.data, msg.data, out.len);

  if (_tap != nullptr) {
    _tap("RX", out, true);
  }
  return true;
}

}  // namespace cansimple

// ============================ bus diagnostics ==============================

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
