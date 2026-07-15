/* ===========================================================================
 *  can_utilities / ESP32 TWAI bridge
 *
 *  This sketch turns an ESP32 into a CANSimple serial bridge for the Python
 *  GUI in GUI/live_plotter.py.
 *
 *  Serial input from the GUI:
 *    A   -> clear errors + arm closed loop
 *    I   -> idle
 *    M   -> motor calibration
 *    C   -> clear errors
 *    E   -> estop
 *    T<x> -> torque command in Nm
 *    V<x> -> velocity command in rad/s
 *    P<x> -> position command in rad
 *
 *  Serial output to the GUI:
 *    t=<ms> #<n> mode=<n> tgt=<value> Iq=<A> vel=<rad/s> pos=<rad> Vbus=<V> RUN
 *
 *  CANSimple addressing:
 *    arbitration_id = (node_id << 5) | command_id
 * ===========================================================================*/

#include <Arduino.h>
#include "driver/twai.h"

// ---------------- configuration --------------------------------------------
constexpr uint8_t  TARGET_NODE_ID = 0;
constexpr uint32_t CAN_BAUD       = 500000;  // must match CFG_CAN_BAUD in board_config.h

// ESP32 TWAI controller pins. Adjust to match your board and transceiver.
constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_4;

constexpr uint8_t potPin = GPIO_NUM_34; // Use an appropriate GPIO pin for your ESP32 board
constexpr int   POT_ADC_MAX        = 4095;  // analogReadResolution default (12-bit)
constexpr float POT_VEL_MAX_RAD_S  = 50.0f; // full deflection on either side maps to +/- this
constexpr int   POT_CHANGE_DEADBAND = 10;   // ignore changes smaller than this (ADC noise)

// Ohm-meter reading (160-4.3k ohm travel, resting at 3.3k ohm) only gives an
// ESTIMATE of the rest ADC value -- it assumes the measured resistance span
// maps linearly onto the full 0..POT_ADC_MAX swing, which depends on the
// pot's true total resistance and exact divider wiring, neither of which is
// known here. Used only as the startup default; press 'Z' with the pot at
// physical rest to capture the real value into g_pot_adc_rest instead.
constexpr float POT_OHM_MIN  = 160.0f;
constexpr float POT_OHM_MAX  = 4300.0f;
constexpr float POT_OHM_REST = 3300.0f;
constexpr int   POT_ADC_MIN  = 0;
constexpr int   POT_ADC_MAX_TRAVEL = POT_ADC_MAX;
constexpr int   POT_ADC_REST_DEFAULT = (int)((POT_OHM_REST - POT_OHM_MIN) / (POT_OHM_MAX - POT_OHM_MIN) * POT_ADC_MAX + 0.5f);
constexpr int   POT_REST_DEADBAND_ADC = 100; // +/- window around rest that snaps to 0 rad/s

int g_pot_adc_rest = POT_ADC_REST_DEFAULT; // recalibrated at runtime via 'Z'

int  potValue = 0;      // Variable to store the read value
int  lastPotValue = 0;  // Variable to store the last sent value
bool potInitialized = false; // first sample bypasses the delta filters

// ---------------- CANSimple command ids ------------------------------------
enum Cmd : uint8_t {
  CMD_HEARTBEAT        = 0x001,
  CMD_ESTOP            = 0x002,
  CMD_SET_AXIS_STATE   = 0x007,
  CMD_GET_ENCODER_EST  = 0x009,
  CMD_SET_CTRL_MODE    = 0x00B,
  CMD_SET_INPUT_POS    = 0x00C,
  CMD_SET_INPUT_VEL    = 0x00D,
  CMD_SET_INPUT_TORQUE = 0x00E,
  CMD_SET_LIMITS       = 0x00F,
  CMD_GET_IQ           = 0x014,
  CMD_GET_BUS_VI       = 0x017,
  CMD_CLEAR_ERRORS     = 0x018,
  CMD_MOTOR_CAL        = 0x004,
};

enum AxisState : uint8_t {
  AXIS_IDLE        = 1,
  AXIS_MOTOR_CAL   = 4,
  AXIS_CLOSED_LOOP = 8,
};

enum ControlMode : uint8_t {
  CTRL_VOLTAGE  = 0,
  CTRL_TORQUE   = 1,
  CTRL_VELOCITY = 2,
  CTRL_POSITION = 3,
};

enum InputMode : uint8_t {
  INPUT_PASSTHROUGH = 1,
};

struct TelemetryState {
  uint32_t axis_error = 0;
  uint8_t heartbeat_state = AXIS_IDLE;
  uint32_t last_heartbeat_ms = 0;
  float pos_rad = 0.0f;
  float vel_rad_s = 0.0f;
  float iq_setpoint_a = 0.0f;
  float iq_measured_a = 0.0f;
  float vbus_v = 0.0f;
  float ibus_a = 0.0f;
  bool have_encoder = false;
  bool have_iq = false;
  bool have_vbus = false;
};

struct BridgeState {
  TelemetryState telemetry;
  uint8_t control_mode = CTRL_TORQUE;
  uint8_t input_mode = INPUT_PASSTHROUGH;
  float target_value = 0.0f;
  uint32_t sample_index = 0;
  uint32_t last_telemetry_ms = 0;
  bool link_up = false;   // true once a heartbeat from TARGET_NODE_ID has been seen
};

static BridgeState g_state;
static uint64_t    g_seen_nodes  = 0;      // bitmask of node ids observed on the bus
static bool        g_verbose_can = false;  // toggle with 'D' serial command

// ---------------- low-level helpers ----------------------------------------
static uint32_t canId(uint8_t cmd) {
  return ((uint32_t)TARGET_NODE_ID << 5) | cmd;
}

static void putF32(uint8_t* buffer, float value) {
  memcpy(buffer, &value, sizeof(float));
}

static void putU32(uint8_t* buffer, uint32_t value) {
  memcpy(buffer, &value, sizeof(uint32_t));
}

static uint32_t getU32(const uint8_t* buffer) {
  uint32_t value;
  memcpy(&value, buffer, sizeof(uint32_t));
  return value;
}

static float getF32(const uint8_t* buffer) {
  float value;
  memcpy(&value, buffer, sizeof(float));
  return value;
}

static const char* cmdName(uint8_t cmd) {
  switch (cmd) {
    case CMD_HEARTBEAT:        return "HEARTBEAT";
    case CMD_ESTOP:             return "ESTOP";
    case CMD_SET_AXIS_STATE:    return "SET_AXIS_STATE";
    case CMD_GET_ENCODER_EST:   return "GET_ENCODER_EST";
    case CMD_SET_CTRL_MODE:     return "SET_CTRL_MODE";
    case CMD_SET_INPUT_POS:     return "SET_INPUT_POS";
    case CMD_SET_INPUT_VEL:     return "SET_INPUT_VEL";
    case CMD_SET_INPUT_TORQUE:  return "SET_INPUT_TORQUE";
    case CMD_SET_LIMITS:        return "SET_LIMITS";
    case CMD_GET_IQ:            return "GET_IQ";
    case CMD_GET_BUS_VI:        return "GET_BUS_VI";
    case CMD_CLEAR_ERRORS:      return "CLEAR_ERRORS";
    default:                    return "UNKNOWN";
  }
}

static void logFrame(const char* dir, uint32_t id, uint8_t node, uint8_t cmd,
                      const uint8_t* data, uint8_t len) {
  Serial.print("[CAN "); Serial.print(dir); Serial.print("] id=0x");
  Serial.print(id, HEX);
  Serial.print(" node="); Serial.print(node);
  Serial.print(" cmd=0x"); Serial.print(cmd, HEX);
  Serial.print(" ("); Serial.print(cmdName(cmd)); Serial.print(")");
  Serial.print(" len="); Serial.print(len);
  if (len > 0 && data != nullptr) {
    Serial.print(" data=");
    for (uint8_t i = 0; i < len; i++) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX);
      Serial.print(' ');
    }
  }
  Serial.println();
}

// Logs every new node id seen on the bus, matched or not: the fastest way to
// spot a node-id mismatch between this bridge (TARGET_NODE_ID) and the
// ODrive's CFG_CAN_NODE_ID without having to decode raw arbitration ids.
static void noteNodeSeen(uint8_t node) {
  const uint64_t bit = (uint64_t)1 << (node & 0x3F);
  if ((g_seen_nodes & bit) != 0) return;
  g_seen_nodes |= bit;
  Serial.print("[CAN] new node observed on bus: id="); Serial.print(node);
  if (node == TARGET_NODE_ID) {
    Serial.println(" (matches target)");
  } else {
    Serial.println(" (does NOT match TARGET_NODE_ID — check CFG_CAN_NODE_ID on the ODrive board)");
  }
}

static bool sendFrame(uint8_t cmd, const uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = canId(cmd);
  msg.data_length_code = len;
  msg.flags = TWAI_MSG_FLAG_NONE;
  if (data != nullptr && len > 0) {
    memcpy(msg.data, data, len);
  }
  const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  logFrame(err == ESP_OK ? "TX" : "TX-FAIL", msg.identifier, TARGET_NODE_ID, cmd, msg.data, len);
  if (err != ESP_OK) {
    Serial.print("  -> twai_transmit error: "); Serial.println(esp_err_to_name(err));
  }
  return err == ESP_OK;
}

static bool sendTrigger(uint8_t cmd) {
  return sendFrame(cmd, nullptr, 0);
}

static bool setAxisState(uint8_t state) {
  uint8_t data[8] = {0};
  putU32(data, state);
  return sendFrame(CMD_SET_AXIS_STATE, data, 8);
}

static bool setControllerMode(uint8_t mode, uint8_t input_mode) {
  uint8_t data[8] = {0};
  putU32(data, mode);
  putU32(data + 4, input_mode);
  g_state.control_mode = mode;
  g_state.input_mode = input_mode;
  return sendFrame(CMD_SET_CTRL_MODE, data, 8);
}

static bool setInputVel(float rad_s, float torque_ff = 0.0f) {
  uint8_t data[8] = {0};
  putF32(data, rad_s / TWO_PI);
  putF32(data + 4, torque_ff);
  g_state.control_mode = CTRL_VELOCITY;
  g_state.target_value = rad_s;
  return sendFrame(CMD_SET_INPUT_VEL, data, 8);
}

static bool setInputTorque(float nm) {
  uint8_t data[8] = {0};
  putF32(data, nm);
  g_state.control_mode = CTRL_TORQUE;
  g_state.target_value = nm;
  return sendFrame(CMD_SET_INPUT_TORQUE, data, 8);
}

static bool setInputPos(float rad) {
  uint8_t data[8] = {0};
  putF32(data, rad / TWO_PI);
  g_state.control_mode = CTRL_POSITION;
  g_state.target_value = rad;
  return sendFrame(CMD_SET_INPUT_POS, data, 8);
}

static bool setLimits(float vel_rad_s, float current_a) {
  uint8_t data[8] = {0};
  putF32(data, vel_rad_s / TWO_PI);
  putF32(data + 4, current_a);
  return sendFrame(CMD_SET_LIMITS, data, 8);
}

static bool clearErrors() {
  return sendTrigger(CMD_CLEAR_ERRORS);
}

static bool eStop() {
  return sendTrigger(CMD_ESTOP);
}

static void initTwai() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR |
                            TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_RECOVERED;
  // Default rx_queue_len is only 5: the ODrive's cyclic timers (heartbeat,
  // encoder, Iq, Vbus) all fire together on their first crossing, so it can
  // burst 4+ frames in the same ms right at boot while setup() is still
  // blocking on Serial/TX calls and hasn't started draining the queue yet.
  g_config.rx_queue_len = 32;
  g_config.tx_queue_len = 16;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  while (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[CAN] TWAI driver install failed - check GPIOs, transceiver wiring, and driver availability");
    delay(500);
  }
  Serial.println("[CAN] TWAI driver installed");

  while (twai_start() != ESP_OK) {
    Serial.println("[CAN] TWAI start failed");
    delay(500);
  }
  Serial.print("[CAN] TWAI started @ 500 kbit/s, TX=GPIO"); Serial.print((int)TWAI_TX_PIN);
  Serial.print(" RX=GPIO"); Serial.print((int)TWAI_RX_PIN);
  Serial.print(" target_node="); Serial.println(TARGET_NODE_ID);
}

// Bus-level alerts: catch problems the frame-level logs can't (bit errors,
// error-passive, bus-off) which are the classic symptoms of a bad/missing
// termination resistor, wiring swap, or baud-rate mismatch.
static void pollCanAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK) return;

  if (alerts & TWAI_ALERT_BUS_ERROR) {
    Serial.println("[CAN][WARN] bus error (bit/stuff/crc/form/ack) - check wiring, termination, baud rate match");
  }
  if (alerts & TWAI_ALERT_ERR_PASS) {
    Serial.println("[CAN][WARN] controller entered ERROR-PASSIVE state - link unreliable, check termination/wiring");
  }
  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.println("[CAN][ERROR] BUS-OFF - too many errors, controller disabled itself. Attempting recovery...");
    twai_initiate_recovery();
  }
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    Serial.println("[CAN] bus recovered from BUS-OFF, restarting driver");
    twai_start();
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("[CAN][WARN] RX queue full - frames dropped (bus flooded or ESP32 not draining fast enough)");
  }
}

// Periodic bus health dump: error counters climbing while state stays
// RUNNING is the fingerprint of a marginal link (e.g. missing termination)
// that hasn't gone bus-off yet but is dropping frames intermittently.
static void pollCanStatus() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < 2000) return;
  last = now;

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  Serial.print("[CAN] state=");
  switch (status.state) {
    case TWAI_STATE_STOPPED:    Serial.print("STOPPED");    break;
    case TWAI_STATE_RUNNING:    Serial.print("RUNNING");    break;
    case TWAI_STATE_BUS_OFF:    Serial.print("BUS_OFF");    break;
    case TWAI_STATE_RECOVERING: Serial.print("RECOVERING"); break;
    default:                    Serial.print("?");          break;
  }
  Serial.print(" tx_err=");    Serial.print(status.tx_error_counter);
  Serial.print(" rx_err=");    Serial.print(status.rx_error_counter);
  Serial.print(" tx_failed="); Serial.print(status.tx_failed_count);
  Serial.print(" rx_missed="); Serial.print(status.rx_missed_count);
  Serial.print(" rx_overrun="); Serial.print(status.rx_overrun_count);
  Serial.print(" arb_lost=");  Serial.print(status.arb_lost_count);
  Serial.print(" bus_err=");   Serial.print(status.bus_error_count);
  Serial.print(" link=");      Serial.println(g_state.link_up ? "UP" : "DOWN");
}

static void printHelp() {
  Serial.println("[INFO] Commands: A=arm I=idle M=motor-cal C=clear E=estop T<n>=torque Nm V<n>=velocity rad/s P<n>=position rad D=toggle verbose CAN frame log Z=calibrate pot rest (hold pot at rest, then send Z)");
}

static void onHeartbeat(uint32_t error, uint8_t state) {
  if (!g_state.link_up) {
    g_state.link_up = true;
    Serial.print("[CONN] link ESTABLISHED with node "); Serial.println(TARGET_NODE_ID);
  }
  if (state != g_state.telemetry.heartbeat_state || error != g_state.telemetry.axis_error) {
    Serial.print("[HB] node="); Serial.print(TARGET_NODE_ID);
    Serial.print(" axis_state="); Serial.print(state);
    Serial.print(" error=0x"); Serial.println(error, HEX);
  }
  g_state.telemetry.axis_error = error;
  g_state.telemetry.heartbeat_state = state;
  g_state.telemetry.last_heartbeat_ms = millis();
}

static void onEncoderEstimates(float pos_rev, float vel_rev_s) {
  g_state.telemetry.pos_rad = pos_rev * TWO_PI;
  g_state.telemetry.vel_rad_s = vel_rev_s * TWO_PI;
  g_state.telemetry.have_encoder = true;
}

static void onIq(float iq_setpoint_a, float iq_measured_a) {
  g_state.telemetry.iq_setpoint_a = iq_setpoint_a;
  g_state.telemetry.iq_measured_a = iq_measured_a;
  g_state.telemetry.have_iq = true;
}

static void onBusVi(float vbus_v, float ibus_a) {
  g_state.telemetry.vbus_v = vbus_v;
  g_state.telemetry.ibus_a = ibus_a;
  g_state.telemetry.have_vbus = true;
}

static void handleCanFrame(const twai_message_t& msg) {
  const uint8_t cmd = msg.identifier & 0x1F;
  const uint8_t node = (msg.identifier >> 5) & 0x3F;
  noteNodeSeen(node);

  if (g_verbose_can) {
    logFrame("RX", msg.identifier, node, cmd, msg.data, msg.data_length_code);
  }

  if (node != TARGET_NODE_ID && cmd != CMD_ESTOP) {
    return;
  }

  switch (cmd) {
    case CMD_HEARTBEAT:
      if (msg.data_length_code >= 5) {
        onHeartbeat(getU32(msg.data), msg.data[4]);
      }
      break;

    case CMD_GET_ENCODER_EST:
      if (msg.data_length_code >= 8) {
        onEncoderEstimates(getF32(msg.data), getF32(msg.data + 4));
      }
      break;

    case CMD_GET_IQ:
      if (msg.data_length_code >= 8) {
        onIq(getF32(msg.data), getF32(msg.data + 4));
      }
      break;

    case CMD_GET_BUS_VI:
      if (msg.data_length_code >= 8) {
        onBusVi(getF32(msg.data), getF32(msg.data + 4));
      }
      break;

    default:
      break;
  }
}

static void pollCan() {
  twai_message_t msg = {};
  while (twai_receive(&msg, 0) == ESP_OK) {
    handleCanFrame(msg);
  }
}

static void emitTelemetry() {
  const uint32_t now = millis();
  if (now - g_state.last_telemetry_ms < 100) {
    return;
  }
  g_state.last_telemetry_ms = now;

  const bool hb_recent = (now - g_state.telemetry.last_heartbeat_ms) < 500;
  const bool run_state = hb_recent && g_state.telemetry.heartbeat_state == AXIS_CLOSED_LOOP && g_state.telemetry.axis_error == 0;

  if (g_state.link_up && !hb_recent) {
    g_state.link_up = false;
    Serial.println("[CONN] link LOST (no heartbeat for 500ms)");
  }

  Serial.print("t="); Serial.print(now);
  Serial.print(" #"); Serial.print(g_state.sample_index++);
  Serial.print(" mode="); Serial.print(g_state.control_mode);

  Serial.print(" tgt=");
  if (g_state.control_mode == CTRL_POSITION) {
    Serial.print(g_state.target_value, 3);
  } else if (g_state.control_mode == CTRL_VELOCITY) {
    Serial.print(g_state.target_value, 3);
  } else {
    Serial.print(g_state.target_value, 3);
  }

  Serial.print(" Iq=");
  Serial.print(g_state.telemetry.iq_measured_a, 3);
  Serial.print(" vel=");
  Serial.print(g_state.telemetry.vel_rad_s, 3);
  Serial.print(" pos=");
  Serial.print(g_state.telemetry.pos_rad, 3);
  Serial.print(" Vbus=");
  Serial.print(g_state.telemetry.vbus_v, 1);
  Serial.print(run_state ? " RUN" : " SAFE");
  if (g_state.telemetry.axis_error != 0) {
    Serial.print(" [FAULT 0x");
    Serial.print(g_state.telemetry.axis_error, HEX);
    Serial.print("]");
  }
  Serial.println();
}

static void processCommandLine(const char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  const char command = (char)toupper((unsigned char)line[0]);
  const char* arg = line + 1;
  while (*arg == ' ' || *arg == '\t' || *arg == '=') {
    ++arg;
  }

  switch (command) {
    case 'A':
      if (clearErrors() && setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH) && setInputVel(0.0f) && setAxisState(AXIS_CLOSED_LOOP)) {
        Serial.println("[OK] armed closed loop");
      } else {
        Serial.println("[ERR] failed to arm");
      }
      break;

    case 'I':
      if (setAxisState(AXIS_IDLE)) {
        Serial.println("[OK] idle");
      } else {
        Serial.println("[ERR] failed to idle");
      }
      break;

    case 'M':
      if (setAxisState(AXIS_MOTOR_CAL)) {
        Serial.println("[OK] motor calibration requested");
      } else {
        Serial.println("[ERR] failed to request motor calibration");
      }
      break;

    case 'C':
      if (clearErrors()) {
        Serial.println("[OK] clear errors sent");
      } else {
        Serial.println("[ERR] clear errors failed");
      }
      break;

    case 'E':
      if (eStop()) {
        Serial.println("[OK] estop sent");
      } else {
        Serial.println("[ERR] estop failed");
      }
      break;

    case 'T': {
      const float torque_nm = strtof(arg, nullptr);
      if (setControllerMode(CTRL_TORQUE, INPUT_PASSTHROUGH) && setInputTorque(torque_nm)) {
        Serial.print("[OK] torque="); Serial.println(torque_nm, 3);
      } else {
        Serial.println("[ERR] torque command failed");
      }
      break;
    }

    case 'V': {
      const float vel_rad_s = strtof(arg, nullptr);
      if (setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH) && setInputVel(vel_rad_s)) {
        Serial.print("[OK] velocity="); Serial.println(vel_rad_s, 3);
      } else {
        Serial.println("[ERR] velocity command failed");
      }
      break;
    }

    case 'P': {
      const float pos_rad = strtof(arg, nullptr);
      if (setControllerMode(CTRL_POSITION, INPUT_PASSTHROUGH) && setInputPos(pos_rad)) {
        Serial.print("[OK] position="); Serial.println(pos_rad, 3);
      } else {
        Serial.println("[ERR] position command failed");
      }
      break;
    }

    case 'D':
      g_verbose_can = !g_verbose_can;
      Serial.print("[OK] verbose CAN frame logging "); Serial.println(g_verbose_can ? "ON" : "OFF");
      break;

    case 'Z': {
      const int old_rest = g_pot_adc_rest;
      g_pot_adc_rest = analogRead(potPin);
      Serial.print("[OK] pot rest calibrated: adc "); Serial.print(old_rest);
      Serial.print(" -> "); Serial.println(g_pot_adc_rest);
      break;
    }

    case '?':
      printHelp();
      break;

    default:
      Serial.print("[ERR] unknown command: ");
      Serial.println(command);
      printHelp();
      break;
  }
}

static void handleSerial() {
  static char buffer[64];
  static size_t index = 0;

  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (index > 0) {
        buffer[index] = '\0';
        processCommandLine(buffer);
        index = 0;
      }
      continue;
    }

    if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}

// ---------------------------------------------------------------------------

// Maps a raw ADC reading to a velocity command, treating g_pot_adc_rest as a
// (non-centered) neutral point: readings within POT_REST_DEADBAND_ADC of
// rest snap to exactly 0 rad/s, and each side of rest is independently
// scaled to its own full +/-POT_VEL_MAX_RAD_S range (the low side spans
// fewer ADC counts than the high side, since rest isn't at the midpoint).
static float potToVelocity(int adc) {
  const int hi_edge = g_pot_adc_rest + POT_REST_DEADBAND_ADC;
  const int lo_edge = g_pot_adc_rest - POT_REST_DEADBAND_ADC;

  if (adc >= lo_edge && adc <= hi_edge) {
    return 0.0f;
  }
  if (adc > hi_edge) {
    const float span = (float)(POT_ADC_MAX_TRAVEL - hi_edge);
    const float frac = span > 0.0f ? (adc - hi_edge) / span : 0.0f;
    return constrain(frac, 0.0f, 1.0f) * POT_VEL_MAX_RAD_S;
  }
  const float span = (float)(lo_edge - POT_ADC_MIN);
  const float frac = span > 0.0f ? (lo_edge - adc) / span : 0.0f;
  return -constrain(frac, 0.0f, 1.0f) * POT_VEL_MAX_RAD_S;
}

void handlePotentiometer() {
  //timer to read the potentiometer value every 100ms
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 100) {
    return;
  }
  lastUpdate = millis();

  // Read the analog value from the potentiometer
  potValue = analogRead(potPin);

  const int delta = abs(potValue - lastPotValue);
  // Always track the latest reading, even on ticks we don't act on. A
  // spring-return pot snapping back from full deflection to rest can easily
  // cross a couple thousand ADC counts within one 100ms poll -- if a large
  // jump like that were rejected as a "spike" and lastPotValue left stale,
  // every subsequent (now-legitimate) reading near rest would ALSO look
  // like a huge jump from that stale reference and get rejected too,
  // permanently latching the last-sent velocity (observed as the target
  // sticking at +/-10 rad/s and never returning to 0). No spike filtering
  // is worth that risk on a motor velocity input -- potToVelocity() already
  // clamps to +/-POT_VEL_MAX_RAD_S, so a single noisy sample costs at most
  // one 100ms tick at the rail, not a permanent lockup.
  lastPotValue = potValue;

  if (!potInitialized) {
    potInitialized = true;
    return; // first sample only seeds lastPotValue, nothing to send yet
  }

  //filter out small changes in potentiometer value to avoid sending too many commands
  if (delta < POT_CHANGE_DEADBAND) {
    return; // No significant change in potentiometer value, skip sending
  }

  const float mappedValue = potToVelocity(potValue);

  // Only (re-)assert velocity mode if we're not already in it -- avoids
  // doubling CAN traffic on every pot tick, and lets the pot naturally
  // resume control after a one-off T<torque>/P<pos> serial command.
  const bool mode_ok = (g_state.control_mode == CTRL_VELOCITY) ||
                        setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH);

  //send the potentiometer value over can as a velocity command
  if (mode_ok && setInputVel(mappedValue)) {
    Serial.print("[OK] pot adc="); Serial.print(potValue);
    Serial.print(" vel="); Serial.print(mappedValue, 3); Serial.println(" rad/s");
  } else {
    Serial.println("[ERR] potentiometer command failed");
  }
}

// ---------------- Arduino setup and loop -----------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  initTwai();

  Serial.println("[INFO] CAN bridge ready for GUI/live_plotter.py");
  printHelp();

  clearErrors();
  setLimits(10.0f, 15.0f);
  setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH);
  setInputVel(0.0f);
}

void loop() {
  handleSerial();
  pollCan();
  pollCanAlerts();
  pollCanStatus();
  emitTelemetry();
  handlePotentiometer();
}