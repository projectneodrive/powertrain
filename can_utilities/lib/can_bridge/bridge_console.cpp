// ============================================================================
//  bridge_console.cpp — see bridge_console.h.
// ============================================================================
#include "bridge_console.h"

#include <ctype.h>
#include <stdlib.h>

#include "log.h"

namespace bridge {
namespace console {
namespace {

Context g_ctx;

// The raw text after the command letters, as typed. Handlers take a float —
// that signature is the firmware's and is not ours to change — but atof("")
// and atof("0") are both 0.0f, and a couple of commands genuinely need to tell
// "D" from "D0". dispatch() points this at the argument before every call.
const char* g_arg = "";
bool argEmpty() { return g_arg == nullptr || g_arg[0] == '\0'; }

// Which banner line a command appears on. The first three are the firmware's
// own groups, kept identical so its commands land where an operator expects;
// GRP_BRIDGE is the extra line for what only exists on this station.
enum Group : uint8_t { GRP_CMDS, GRP_GAINS, GRP_CONFIG, GRP_BRIDGE };

using Handler = void (*)(float);

// ---------------------------------------------------------------------------
//  Acknowledgements — byte-identical to the firmware's io::console helpers, so
//  host tooling parsing "AK ..." does not need to know which port it is on.
// ---------------------------------------------------------------------------
void ackFloat(const char* tag, const char* field, float oldv, float newv,
              uint8_t prec, const char* unit = nullptr) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.print(field); Serial.print(' ');
  Serial.print(oldv, prec);
  Serial.print(" -> ");
  Serial.print(newv, prec);
  if (unit) { Serial.print(' '); Serial.print(unit); }
  Serial.println();
}

void ackMsg(const char* tag, const char* message) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.println(message);
}

// A command that reached the bus but was not accepted by the TWAI controller.
void ackTxFail(const char* tag) {
  Serial.print("AK "); Serial.print(tag);
  Serial.println(": CAN transmit FAILED - see the [CAN] lines above");
}

void ack(const char* tag, bool ok, const char* message) {
  ok ? ackMsg(tag, message) : ackTxFail(tag);
}

// The answer for everything the board can do but CANSimple cannot express.
// Deliberately not silent and deliberately not an error: the operator asked a
// reasonable question and there is a real answer, it is just "other port".
void notOverCan(const char* tag, const char* what) {
  Serial.print("AK "); Serial.print(tag); Serial.print(": ");
  Serial.print(what);
  Serial.println(" is not in the CANSimple command set - use the board's own USB console");
}

// ---------------------------------------------------------------------------
//  Handlers for the FIRMWARE's command set (../include/console_commands.h).
//  Same names as the board's, so the two tables read side by side.
// ---------------------------------------------------------------------------
void cmdArm(float) {
  ack("A", g_ctx.axis->arm(), "armed closed loop");
}

void cmdIdle(float) {
  ack("I", g_ctx.axis->idle(), "idle");
}

void cmdVelocity(float v) {
  const float old = g_ctx.state->c.input_vel;
  if (g_ctx.axis->setControllerMode(odcan::CTRL_VELOCITY, INPUT_PASSTHROUGH) &&
      g_ctx.axis->setVelocity(v)) {
    ackFloat("V", "vel", old, v, 2, "rad/s");
  } else {
    ackTxFail("V");
  }
}

void cmdTorque(float v) {
  const float old = g_ctx.state->c.input_torque;
  if (g_ctx.axis->setControllerMode(odcan::CTRL_TORQUE, INPUT_PASSTHROUGH) &&
      g_ctx.axis->setTorque(v)) {
    ackFloat("T", "torque", old, v, 2, "Nm");
  } else {
    ackTxFail("T");
  }
}

void cmdPosition(float v) {
  const float old = g_ctx.state->c.input_pos;
  if (g_ctx.axis->setControllerMode(odcan::CTRL_POSITION, INPUT_PASSTHROUGH) &&
      g_ctx.axis->setPosition(v)) {
    ackFloat("X", "pos", old, v, 3, "rad");
  } else {
    ackTxFail("X");
  }
}

void cmdCharacterise(float) {
  // The board maps Set_Axis_State(MOTOR_CALIBRATION) onto the same R/L
  // measurement its 'M' triggers — see rxSetAxisState in odrive_can.cpp.
  ack("M", g_ctx.axis->characterise(), "characterise requested");
}

#if SENSOR_TYPE == SENSOR_TYPE_HALL
void cmdHallCal(float) {
  // AXIS_ENC_OFFSET_CAL exists in the ODrive state vocabulary, but the
  // firmware's rxSetAxisState does not act on it: hall calibration is a
  // blocking commissioning sequence, started from the board's own console.
  notOverCan("H", "hall-angle calibration");
}
#endif

void cmdClearErrors(float) {
  ack("C", g_ctx.axis->clearErrors(), "clear-errors requested");
}

// ---- Velocity PID ----------------------------------------------------------
// Set_Vel_Gains (0x01B) carries P and I in one frame and has no D field, so
// every one of these resends the pair from the cached state.
void cmdVelP(float v) {
  const float old = g_ctx.state->c.vel_gain;
  g_ctx.state->c.vel_gain = v;
  if (g_ctx.axis->applyVelGains()) ackFloat("KP", "vel_gain", old, v, 4);
  else                             ackTxFail("KP");
}

void cmdVelI(float v) {
  const float old = g_ctx.state->c.vel_int_gain;
  g_ctx.state->c.vel_int_gain = v;
  if (g_ctx.axis->applyVelGains()) ackFloat("KI", "vel_int_gain", old, v, 4);
  else                             ackTxFail("KI");
}

void cmdVelD(float) {
  notOverCan("KD", "the velocity PID's D term");
}

void cmdVelReapply(float) {
  ack("K", g_ctx.axis->applyVelGains(), "reapply vel gains");
}

// ---- Current PID -----------------------------------------------------------
// No CANSimple command touches the current loop at all.
void cmdCurP(float)       { notOverCan("JP", "the current PID"); }
void cmdCurI(float)       { notOverCan("JI", "the current PID"); }
void cmdCurD(float)       { notOverCan("JD", "the current PID"); }
void cmdCurReapply(float) { notOverCan("J",  "the current PID"); }

// ---- Position PID ----------------------------------------------------------
// Set_Pos_Gain (0x01A) carries the P term only.
void cmdPosP(float v) {
  const float old = g_ctx.state->c.pos_gain;
  g_ctx.state->c.pos_gain = (v > 0.0f) ? v : 0.0f;
  if (g_ctx.axis->applyPosGain()) ackFloat("PP", "pos_p", old, g_ctx.state->c.pos_gain, 4);
  else                            ackTxFail("PP");
}

void cmdPosI(float) { notOverCan("PI", "the position PID's I term"); }
void cmdPosD(float) { notOverCan("PD", "the position PID's D term"); }

void cmdPosReapply(float) {
  ack("P", g_ctx.axis->applyPosGain(), "reapply position gain");
}

// ---- Runtime limits --------------------------------------------------------
// Clamped to the same hard ceilings the firmware clamps to — from the same
// header, so a remote client cannot request a dangerous value at either end.
// Set_Limits carries velocity AND current together, hence the cached pair.
void cmdLimitCurrent(float v) {
  const float old = g_ctx.state->c.current_limit;
  g_ctx.state->c.current_limit = constrain(v, 0.0f, (float)CFG_CURRENT_LIMIT_MAX);
  if (g_ctx.axis->applyLimits())
    ackFloat("LC", "current_limit", old, g_ctx.state->c.current_limit, 2, "A");
  else
    ackTxFail("LC");
}

void cmdLimitVelocity(float v) {
  const float old = g_ctx.state->c.vel_limit;
  g_ctx.state->c.vel_limit = constrain(v, 0.0f, (float)CFG_VEL_LIMIT_MAX);
  if (g_ctx.axis->applyLimits())
    ackFloat("LV", "vel_limit", old, g_ctx.state->c.vel_limit, 2, "rad/s");
  else
    ackTxFail("LV");
}

void cmdLimitHelp(float) {
  Serial.println("AK L?: use LC<A> or LV<rad/s>");
}

void cmdPosGain(float v) {
  const float old = g_ctx.state->c.pos_gain;
  g_ctx.state->c.pos_gain = (v > 0.0f) ? v : 0.0f;
  if (g_ctx.axis->applyPosGain())
    ackFloat("G", "pos_gain", old, g_ctx.state->c.pos_gain, 4);
  else
    ackTxFail("G");
}

// ---------------------------------------------------------------------------
//  Q — the config line, same "cfg key=value" shape the GUI's config page reads
//  from the board.
//
//  READ THIS BEFORE TRUSTING IT. On the board, Q reports the LIVE values out of
//  the motor object. Here there is no way to ask: CANSimple has no
//  configuration read-back. So the settable fields are what this station last
//  COMMANDED, and the rest are the compile-time CFG_* constants the firmware
//  was built with. They match reality unless somebody also changed something
//  over the board's USB console, or the board is running a different build.
// ---------------------------------------------------------------------------
void cmdDumpConfig(float) {
  const Commanded& c = g_ctx.state->c;   // named by the schema's bridge exprs
  Serial.print("cfg");
#define CONFIG_PARAM(key, label, unit, cmd, prec, demo, fw, br)   Serial.print(" " #key "="); Serial.print((float)(br), prec);
#include "config_schema.h"
#undef CONFIG_PARAM
  Serial.println(" src=bridge");    // so the GUI can tell where this came from
}

// ---------------------------------------------------------------------------
//  Handlers for the STATION's own commands (bridge_commands.h).
// ---------------------------------------------------------------------------
void cmdEstop(float) {
  ack("E", g_ctx.axis->estop(), "estop sent");
}

void cmdReboot(float) {
  ack("R", g_ctx.axis->reboot(), "reboot requested (link will drop)");
}

void cmdFaults(float) {
  const Measured& m = g_ctx.state->m;
  // Print what we already hold, then ask for a refresh: the replies arrive
  // asynchronously, so the numbers below are from the previous request.
  // The axis word is decoded from the shared axis_vocab.h table — the per
  // subsystem words are ODrive's own MotorError/EncoderError/ControllerError
  // enums, which this firmware does not populate with named bits, so they stay
  // hex rather than being given names they do not have.
  char names[128];
  axisnames::errors(m.axis_error, names, sizeof(names));
  Serial.printf("AK F: axis=0x%lX %s motor=0x%lX encoder=0x%lX controller=0x%lX (refreshing...)\n",
                (unsigned long)m.axis_error, names, (unsigned long)m.motor_error,
                (unsigned long)m.encoder_error, (unsigned long)m.controller_error);
  if (!g_ctx.axis->requestErrors()) ackTxFail("F");
}

// D<n> — event log level. Bare "D" reports the current one rather than
// changing it, since atof("") is 0 and silently dropping to errors-only is the
// last thing somebody typing "D" to see what it does wants.
void cmdLogLevel(float v) {
  if (argEmpty()) {
    Serial.print("AK D: log level ");
    Serial.print(logx::levelName(logx::level()));
    Serial.println(" (D0=errors D1=+warnings D2=+state changes D3=+CAN frame trace)");
    return;
  }
  const int requested = (int)(v + 0.5f);
  const logx::Level lv = (logx::Level)constrain(requested, 0, (int)logx::LVL_DEBUG);
  const logx::Level old = logx::level();
  logx::setLevel(lv);
  Serial.print("AK D: log level ");
  Serial.print(logx::levelName(old));
  Serial.print(" -> ");
  Serial.println(logx::levelName(lv));
}

void cmdPotRest(float) {
  // Blocks for ~64 ms while it averages. That is well inside what the RX queue
  // absorbs (BRIDGE_RX_QUEUE_LEN holds ~2.5x the frames the board sends in that
  // time), and it is an explicit operator action, not something on the hot path.
  const int old_rest = g_ctx.pot->rest();
  const pot::RestMeasurement m = g_ctx.pot->calibrateRest();
  Serial.printf("AK Z: pot_rest %d -> %d (spread %d, %s)\n", old_rest, m.adc, m.spread,
                m.ok() ? "steady" : (!m.stable ? "UNSTEADY - hold it still and repeat"
                                               : "near a rail - check the wiring"));
}

void cmdHelp(float) { printBanner(); }

// ---------------------------------------------------------------------------
//  The dispatch table: the firmware's commands, then this station's.
// ---------------------------------------------------------------------------
struct Command {
  char        key;
  char        sub;        // 0 = single-letter command
  uint8_t     group;
  const char* help;
  Handler     fn;
  bool        bridge_only;
};

const Command COMMANDS[] = {
#define CONSOLE_CMD(key, sub, group, help, handler) { key, sub, group, help, handler, false },
#include "console_commands.h"
#undef CONSOLE_CMD

#define BRIDGE_CMD(key, sub, group, help, handler)  { key, sub, group, help, handler, true  },
#include "bridge_commands.h"
#undef BRIDGE_CMD
};
constexpr size_t COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

// Two passes, in this order — the same rule as the firmware's dispatch:
//   1. exact key+sub  -> two-letter commands ("KP0.5"), argument at line+2
//   2. key + sub == 0 -> single-letter catch-all ("V5", "K"), argument at line+1
// The catch-all is what lets an unrecognised second letter still reach the
// family's default branch.
void dispatch(const char* line) {
  const char key = (char)toupper((unsigned char)line[0]);
  const char sub = line[1] ? (char)toupper((unsigned char)line[1]) : 0;

  if (sub) {
    for (size_t i = 0; i < COMMAND_COUNT; i++) {
      if (COMMANDS[i].key == key && COMMANDS[i].sub == sub) {
        g_arg = line + 2;
        COMMANDS[i].fn((float)atof(g_arg));
        return;
      }
    }
  }
  for (size_t i = 0; i < COMMAND_COUNT; i++) {
    if (COMMANDS[i].key == key && COMMANDS[i].sub == 0) {
      g_arg = line + 1;
      COMMANDS[i].fn((float)atof(g_arg));
      return;
    }
  }

  Serial.print("AK ?: unknown '"); Serial.print(line[0]);
  Serial.println("'");
}

void printGroup(uint8_t group, const char* prefix) {
  Serial.print(prefix);
  bool first = true;
  for (size_t i = 0; i < COMMAND_COUNT; i++) {
    if (COMMANDS[i].group != group || !COMMANDS[i].help[0]) continue;
    if (!first) Serial.print(" | ");
    Serial.print(COMMANDS[i].help);
    first = false;
  }
  Serial.println();
}

// The two tables are built independently, so nothing prevents bridge_commands.h
// from shadowing a firmware key — and a shadowed key would silently do the
// wrong thing on one of the two ports, which is the exact class of bug this
// whole arrangement exists to kill. Checked once at boot, loudly.
void checkForKeyCollisions() {
  for (size_t i = 0; i < COMMAND_COUNT; i++) {
    if (!COMMANDS[i].bridge_only) continue;
    for (size_t j = 0; j < COMMAND_COUNT; j++) {
      if (COMMANDS[j].bridge_only || COMMANDS[j].key != COMMANDS[i].key) continue;
      LOG_E("SYS", "command key '%c' in bridge_commands.h shadows one in the firmware's "
                   "console_commands.h - pick another letter",
            COMMANDS[i].key);
    }
  }
}

}  // namespace

// ---------------------------------------------------------------------------
void begin(const Context& ctx) {
  g_ctx = ctx;
  checkForKeyCollisions();
}

void printBanner() {
  printGroup(GRP_CMDS,   "Serial cmds: ");
  printGroup(GRP_GAINS,  "  gains:     ");
  printGroup(GRP_CONFIG, "  config:    ");
  printGroup(GRP_BRIDGE, "  bridge:    ");
}

void poll() {
  static char   buffer[32];
  static size_t index = 0;

  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (index > 0) {
        buffer[index] = '\0';
        dispatch(buffer);
        index = 0;
      }
      continue;   // empty lines are swallowed, keep draining
    }
    if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}

}  // namespace console
}  // namespace bridge
