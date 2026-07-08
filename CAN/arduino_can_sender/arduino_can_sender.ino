/* ===========================================================================
 *  arduino_can_sender.ino
 *  Example CAN master for the SimpleFOC / MKS-ODrive board.
 *  Speaks the ODrive "CANSimple" protocol, so it works with the firmware in
 *  this repo (Neodrive_test) and with a stock ODrive.
 *
 *  HARDWARE
 *    Arduino Uno / Nano  +  MCP2515 CAN module (blue board, 8 MHz or 16 MHz).
 *    Library: "mcp_can" by coryjfowler  (Arduino IDE -> Library Manager).
 *
 *  WIRING (Uno / Nano)
 *    MCP2515  SCK  -> D13      MCP2515  SI(MOSI) -> D11
 *             SO(MISO) -> D12           CS       -> D10
 *             INT  -> D2                VCC      -> 5V
 *                                       GND      -> GND
 *    CAN bus: module CANH <-> board CANH,  module CANL <-> board CANL,
 *             GND shared, and a 120 ohm resistor across CANH/CANL at BOTH
 *             ends of the bus (the MCP2515 blue board usually already has one).
 *
 *  THE ADDRESSING SCHEME (important)
 *    Every CANSimple frame uses an 11-bit standard id built as:
 *        arbitration_id = (node_id << 5) | command_id
 *    So node 0 -> ids 0x000..0x01F, node 1 -> 0x020..0x03F, etc.
 *    Data payloads are little-endian; floats are IEEE-754 32-bit.
 * ===========================================================================*/
#include <SPI.h>
#include <mcp_can.h>

// ---------------- configuration (must match the board) ----------------------
const uint8_t NODE_ID  = 0;            // = CFG_CAN_NODE_ID in board_config.h
const long    CAN_BAUD = CAN_100KBPS;  // = CFG_CAN_BAUD (100 kbit/s)
const uint8_t MCP_CLK  = MCP_8MHZ;     // set MCP_16MHZ if your module has a 16 MHz crystal
const uint8_t CS_PIN   = 10;

MCP_CAN CAN(CS_PIN);

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
static void putF32(uint8_t* b, float f)   { memcpy(b, &f, 4); }   // AVR is little-endian
static void putI32(uint8_t* b, int32_t v) { memcpy(b, &v, 4); }

static void sendFrame(uint8_t cmd, uint8_t* data, uint8_t len) {
  CAN.sendMsgBuf(canId(cmd), 0 /* standard id */, len, data);
}
static void sendTrigger(uint8_t cmd) {          // zero-length command
  uint8_t dummy[1] = {0};
  CAN.sendMsgBuf(canId(cmd), 0, 0, dummy);
}

// ---------------- high-level commands (the whole scheme) --------------------
void setAxisState(int32_t state) {
  uint8_t d[8] = {0}; putI32(d, state);         sendFrame(CMD_SET_AXIS_STATE, d, 8);
}
void setControllerMode(int32_t mode, int32_t input_mode) {
  uint8_t d[8] = {0}; putI32(d, mode); putI32(d + 4, input_mode);
  sendFrame(CMD_SET_CTRL_MODE, d, 8);
}
void setInputVel(float rev_s, float torque_ff = 0.0f) {
  uint8_t d[8]; putF32(d, rev_s); putF32(d + 4, torque_ff);
  sendFrame(CMD_SET_INPUT_VEL, d, 8);
}
void setInputTorque(float nm) {
  uint8_t d[8] = {0}; putF32(d, nm);            sendFrame(CMD_SET_INPUT_TORQUE, d, 8);
}
void setInputPos(float rev) {
  uint8_t d[8] = {0}; putF32(d, rev);           sendFrame(CMD_SET_INPUT_POS, d, 8);
}
void setLimits(float vel_rev_s, float current_a) {
  uint8_t d[8]; putF32(d, vel_rev_s); putF32(d + 4, current_a);
  sendFrame(CMD_SET_LIMITS, d, 8);
}
void eStop()       { sendTrigger(CMD_ESTOP); }
void clearErrors() { sendTrigger(CMD_CLEAR_ERRORS); }

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLK) != CAN_OK) {
    Serial.println("MCP2515 init failed - check wiring / crystal (8 vs 16 MHz)");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);              // required to actually transmit
  Serial.println("CAN ready. Commanding the board...");

  clearErrors();                        // start from a clean state
  setLimits(10.0f, 15.0f);             // 10 rev/s max, 15 A max
  setAxisState(AXIS_CLOSED_LOOP);       // arm (this board also boots armed)
  setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH);
  setInputVel(2.0f);                    // spin at 2 rev/s
}

uint32_t t_reverse = 0;
float    vel = 2.0f;

void loop() {
  // ---- read telemetry the board streams back ----
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint32_t id; uint8_t len, buf[8];
    CAN.readMsgBuf(&id, &len, buf);
    switch (id & 0x1F) {                // command id = low 5 bits
      case CMD_HEARTBEAT: {
        uint32_t err; memcpy(&err, buf, 4);
        Serial.print("HB  state="); Serial.print(buf[4]);
        Serial.print("  err=0x");   Serial.println(err, HEX);
      } break;
      case CMD_ENCODER_EST: {
        float pos, v; memcpy(&pos, buf, 4); memcpy(&v, buf + 4, 4);
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
