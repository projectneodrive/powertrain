/* ===========================================================================
 *  esp32_twai_sender.ino
 *  ESP32 native CAN master for the SimpleFOC / MKS-ODrive board.
 *  Uses the ESP32 built-in TWAI controller plus an external CAN transceiver
 *  such as the CJMCU-230. Speaks the ODrive "CANSimple" protocol, so it works
 *  with the firmware in this repo and with a stock ODrive.
 *
 *  HARDWARE
 *    ESP32 + CAN transceiver module (CJMCU-230 / SN65HVD230 / MCP2551, etc.).
 *    The ESP32 TWAI TX/RX pins connect to the transceiver TXD/RXD pins.
 *
 *    Example wiring:
 *      ESP32 TWAI TX pin -> transceiver TXD
 *      ESP32 TWAI RX pin -> transceiver RXD
 *      Transceiver CANH/CANL -> board CANH/CANL
 *      GND shared between ESP32, transceiver, and motor controller
 *
 *  IMPORTANT
 *    - TWAI is the native CAN controller in the ESP32.
 *    - The CJMCU-230 is only the physical layer transceiver; it does not
 *      replace the TWAI controller.
 *    - Use a 3.3 V compatible transceiver or level shifting as required.
 *
 *  CANSimple ADDRESSING
 *    arbitration_id = (node_id << 5) | command_id
 *    Payloads are little-endian; floats are IEEE-754 32-bit.
 * ===========================================================================*/

#include <Arduino.h>
#include "driver/twai.h"

// ---------------- configuration (must match the board) ----------------------
const uint8_t NODE_ID  = 0;            // = CFG_CAN_NODE_ID in board_config.h
const uint32_t CAN_BAUD = 100000;       // = CFG_CAN_BAUD (100 kbit/s)

// Pick GPIOs that are free on your ESP32 board.
// These are the TWAI controller pins, not the CANH/CANL bus wires.
const gpio_num_t TWAI_TX_PIN = GPIO_NUM_5;
const gpio_num_t TWAI_RX_PIN = GPIO_NUM_4;

// ---------------- CANSimple command ids (low 5 bits of the id) --------------
enum {
  CMD_HEARTBEAT        = 0x001,   // <- from board: state + error flags
  CMD_ESTOP            = 0x002,   // -> board: emergency stop
  CMD_SET_AXIS_STATE   = 0x007,   // -> board: idle / closed-loop
  CMD_ENCODER_EST      = 0x009,   // <- from board: pos (rev), vel (rev/s)
  CMD_SET_CTRL_MODE    = 0x00B,   // -> board: control mode + input mode
  CMD_SET_INPUT_POS    = 0x00C,   // -> board: position (rev)
  CMD_SET_INPUT_VEL    = 0x00D,   // -> board: velocity (rev/s) [+ torque FF]
  CMD_SET_INPUT_TORQUE = 0x00E,   // -> board: torque (Nm)
  CMD_SET_LIMITS       = 0x00F,   // -> board: vel limit (rev/s), current limit (A)
  CMD_GET_IQ           = 0x014,   // <- from board: Iq setpoint / measured (A)
  CMD_GET_BUS_VI       = 0x017,   // <- from board: bus voltage (V) / current (A)
  CMD_CLEAR_ERRORS     = 0x018,   // -> board: clear latched errors
};

// AxisState values
enum { AXIS_IDLE = 1, AXIS_CLOSED_LOOP = 8 };
// ControlMode values
enum { CTRL_VOLTAGE = 0, CTRL_TORQUE = 1, CTRL_VELOCITY = 2, CTRL_POSITION = 3 };
// InputMode values
enum { INPUT_PASSTHROUGH = 1 };

// ---------------- low-level helpers -----------------------------------------
static uint32_t canId(uint8_t cmd) { return ((uint32_t)NODE_ID << 5) | cmd; }
static void putF32(uint8_t* b, float f) { memcpy(b, &f, 4); }
static void putI32(uint8_t* b, int32_t v) { memcpy(b, &v, 4); }

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

// ---------------- high-level commands (the whole scheme) --------------------
void setAxisState(int32_t state) {
  uint8_t d[8] = {0};
  putI32(d, state);
  sendFrame(CMD_SET_AXIS_STATE, d, 8);
}

void setControllerMode(int32_t mode, int32_t input_mode) {
  uint8_t d[8] = {0};
  putI32(d, mode);
  putI32(d + 4, input_mode);
  sendFrame(CMD_SET_CTRL_MODE, d, 8);
}

void setInputVel(float rev_s, float torque_ff = 0.0f) {
  uint8_t d[8];
  putF32(d, rev_s);
  putF32(d + 4, torque_ff);
  sendFrame(CMD_SET_INPUT_VEL, d, 8);
}

void setInputTorque(float nm) {
  uint8_t d[8] = {0};
  putF32(d, nm);
  sendFrame(CMD_SET_INPUT_TORQUE, d, 8);
}

void setInputPos(float rev) {
  uint8_t d[8] = {0};
  putF32(d, rev);
  sendFrame(CMD_SET_INPUT_POS, d, 8);
}

void setLimits(float vel_rev_s, float current_a) {
  uint8_t d[8];
  putF32(d, vel_rev_s);
  putF32(d + 4, current_a);
  sendFrame(CMD_SET_LIMITS, d, 8);
}

void eStop() { sendTrigger(CMD_ESTOP); }
void clearErrors() { sendTrigger(CMD_CLEAR_ERRORS); }

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

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  initTwai();
  Serial.println("TWAI ready. Commanding the board...");

  clearErrors();                    // start from a clean state
  setLimits(10.0f, 15.0f);          // 10 rev/s max, 15 A max
  setAxisState(AXIS_CLOSED_LOOP);   // arm: the board boots SAFE and needs this
  delay(2000);                      // let the one-time arm calibration finish
  setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH);
  setInputVel(2.0f);                // spin at 2 rev/s
}

uint32_t t_reverse = 0;
float vel = 2.0f;

void loop() {
  // ---- read telemetry the board streams back ----
  twai_message_t msg = {};
  while (twai_receive(&msg, 0) == ESP_OK) {
    switch (msg.identifier & 0x1F) {  // command id = low 5 bits
      case CMD_HEARTBEAT: {
        uint32_t err;
        memcpy(&err, msg.data, 4);
        Serial.print("HB  state="); Serial.print(msg.data[4]);
        Serial.print("  err=0x");   Serial.println(err, HEX);
      } break;
      case CMD_ENCODER_EST: {
        float pos, v;
        memcpy(&pos, msg.data, 4);
        memcpy(&v, msg.data + 4, 4);
        Serial.print("pos="); Serial.print(pos, 3);
        Serial.print(" rev   vel="); Serial.print(v, 3); Serial.println(" rev/s");
      } break;
    }
  }

  // ---- every 3 s reverse direction (shows live velocity commands) ----
  if (millis() - t_reverse > 3000) {
    t_reverse = millis();
    vel = -vel;
    setInputVel(vel);
    Serial.print(">> setInputVel "); Serial.println(vel);
  }
}