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
