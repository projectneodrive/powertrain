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
constexpr uint32_t CAN_BAUD       = 100000;

// ESP32 TWAI controller pins. Adjust to match your board and transceiver.
constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_4;

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
};

static BridgeState g_state;

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

static bool sendFrame(uint8_t cmd, const uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = canId(cmd);
  msg.data_length_code = len;
  msg.flags = TWAI_MSG_FLAG_NONE;
  if (data != nullptr && len > 0) {
    memcpy(msg.data, data, len);
  }
  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
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
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  while (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI init failed - check GPIOs, transceiver wiring, and driver availability");
    delay(500);
  }
  while (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    delay(500);
  }
}

static void printHelp() {
  Serial.println("[INFO] Commands: A=arm I=idle M=motor-cal C=clear E=estop T<n>=torque Nm V<n>=velocity rad/s P<n>=position rad");
}

static void onHeartbeat(uint32_t error, uint8_t state) {
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
  emitTelemetry();
}