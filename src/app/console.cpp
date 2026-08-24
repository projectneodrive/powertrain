// ============================================================================
//  prog_console.cpp — the serial console: command dispatch + the telemetry line.
//
//  The command set lives in include/console_commands.h; this file expands it
//  twice, into the dispatch table and into the help banner. All a new command
//  needs is a handler here and a line there.
// ============================================================================
#include "app.h"
#include "io/io.h"
#include "io/io_motor.h"
#include "util/timers.h"
#include "state.h"
#include "config/motor_config.h"
#include <stdlib.h>
#include <ctype.h>

using namespace odcan;

namespace app {
namespace console {

namespace { uint32_t s_beat = 0; }   // telemetry line counter

namespace {

// Which banner line a command appears on.
enum Group : uint8_t { GRP_CMDS, GRP_GAINS, GRP_CONFIG };

using Handler = void (*)(float);

// ---------------------------------------------------------------------------
//  Command handlers.
//
//  Every one of them acknowledges, showing old -> new. That is what makes the
//  console usable without a second window: you can see whether a setting landed
//  and what it landed on, instead of inferring it from the telemetry.
// ---------------------------------------------------------------------------
void cmdArm(float) {
  bool old = state::axis.armed;
  state::axis.estop = false;
  state::axis.armed = true;
  state::axis.last_setpoint_ms = millis();
  io::console::ackInt("A", "armed", old, state::axis.armed);
}

void cmdIdle(float) {
  bool old = state::axis.armed;
  state::axis.armed = false;
  io::console::ackInt("I", "armed", old, state::axis.armed);
}

void cmdVelocity(float v) {
  float old = state::axis.input_vel;
  state::axis.control_mode = CTRL_VELOCITY;
  state::axis.input_vel    = v;
  state::axis.last_setpoint_ms = millis();
  io::console::ackFloat("V", "vel", old, v, 2, "rad/s");
}

void cmdTorque(float v) {
  float old = state::axis.input_torque;
  state::axis.control_mode = CTRL_TORQUE;
  state::axis.input_torque = v;
  state::axis.last_setpoint_ms = millis();
  io::console::ackFloat("T", "torque", old, v, 2, "Nm");
}

void cmdPosition(float v) {
  float old = state::axis.input_pos;
  state::axis.control_mode = CTRL_POSITION;
  state::axis.input_pos    = v;
  state::axis.last_setpoint_ms = millis();
  io::console::ackFloat("X", "pos", old, v, 3, "rad");
}

void cmdCharacterise(float) {
  state::axis.req_characterise = true;
  io::console::ackMsg("M", "characterise requested");
}

#if SENSOR_TYPE == SENSOR_TYPE_HALL
void cmdHallCal(float) {
  state::req.hall_cal = true;
  io::console::ackMsg("H", "hall-angle calibration requested (moteur désarmé)");
}
#endif

void cmdClearErrors(float) {
  state::axis.req_clear_errors = true;
  state::axis.estop = false;
  io::console::ackMsg("C", "clear-errors requested");
}

// ---- PID gains --------------------------------------------------------------
//  The setters differ only in the field they write, the name the ack shows and
//  the decimals it shows it to, so they are generated. Each raises its loop's
//  req_* flag, which is what makes app::control push the whole trio into
//  SimpleFOC on its next scan (a gain is never applied from this task).
//
//  The ack NAMES are wire format — host tooling greps "AK KP: vel_gain ..." —
//  so they are spelled out here rather than derived from the field name.
// ---------------------------------------------------------------------------
#define GAIN_SETTER(fn, tag, field, name, prec, reqflag)   void fn(float v) {                                         float old = state::axis.field;                           state::axis.field = v;                                   io::console::ackFloat(tag, name, old, v, prec);          state::axis.reqflag = true;                            }
#define GAIN_REAPPLY(fn, tag, msg, reqflag)   void fn(float) { io::console::ackMsg(tag, msg); state::axis.reqflag = true; }

GAIN_SETTER(cmdVelP, "KP", vel_gain,     "vel_gain",     4, req_vel_gains)
GAIN_SETTER(cmdVelI, "KI", vel_int_gain, "vel_int_gain", 4, req_vel_gains)
GAIN_SETTER(cmdVelD, "KD", vel_d_gain,   "vel_d_gain",   5, req_vel_gains)
GAIN_REAPPLY(cmdVelReapply, "K", "reapply vel gains", req_vel_gains)

GAIN_SETTER(cmdCurP, "JP", cur_p_gain,   "cur_p",        4, req_cur_gains)
GAIN_SETTER(cmdCurI, "JI", cur_int_gain, "cur_i",        4, req_cur_gains)
GAIN_SETTER(cmdCurD, "JD", cur_d_gain,   "cur_d",        5, req_cur_gains)
GAIN_REAPPLY(cmdCurReapply, "J", "reapply current gains", req_cur_gains)

GAIN_SETTER(cmdPosI, "PI", pos_int_gain, "pos_i",        4, req_pos_gains)
GAIN_SETTER(cmdPosD, "PD", pos_d_gain,   "pos_d",        5, req_pos_gains)
GAIN_REAPPLY(cmdPosReapply, "P", "reapply position gains", req_pos_gains)

#undef GAIN_REAPPLY
#undef GAIN_SETTER

// Position P is not generated: a negative position gain inverts the feedback
// sign and drives the axis away from its target, so it is clamped at zero.
void cmdPosP(float v) {
  float old = state::axis.pos_gain;
  state::axis.pos_gain = (v > 0.0f) ? v : 0.0f;
  io::console::ackFloat("PP", "pos_p", old, state::axis.pos_gain, 4);
  state::axis.req_pos_gains = true;
}

// ---- Runtime limits ---------------------------------------------------------
// Clamped to a hard ceiling so a remote client can never request a dangerous
// value — see CFG_*_MAX in motor_config.h.
void cmdLimitCurrent(float v) {
  float old = state::axis.current_limit;
  state::axis.current_limit = util::limit(0.0f, v, CFG_CURRENT_LIMIT_MAX);
  io::console::ackFloat("LC", "current_limit", old, state::axis.current_limit, 2, "A");
}
void cmdLimitVelocity(float v) {
  float old = state::axis.vel_limit;
  state::axis.vel_limit = util::limit(0.0f, v, CFG_VEL_LIMIT_MAX);
  io::console::ackFloat("LV", "vel_limit", old, state::axis.vel_limit, 2, "rad/s");
}
void cmdLimitHelp(float) {
  Serial.println("AK L?: use LC<A> or LV<rad/s>");
}

void cmdPosGain(float v) {
  float old = state::axis.pos_gain;
  state::axis.pos_gain = (v > 0.0f) ? v : 0.0f;
  io::console::ackFloat("G", "pos_gain", old, state::axis.pos_gain, 4);
}

// ---------------------------------------------------------------------------
//  Q — dump the live configuration on one line prefixed "cfg " so the GUI can
//  tell it apart from telemetry ("t=..."). Fields, order and precision all come
//  from include/config_schema.h, which the GUI's config table and the ESP32
//  station compile too — so the three cannot disagree about what Q reports.
// ---------------------------------------------------------------------------
void cmdDumpConfig(float) {
  auto& motor = io::motor::motor;   // named by the schema's firmware expressions
  Serial.print("cfg");
#define CONFIG_PARAM(key, label, unit, cmd, prec, demo, fw, br)   Serial.print(" " #key "="); Serial.print((float)(fw), prec);
#include "config_schema.h"
#undef CONFIG_PARAM
  Serial.println();
}

// ---------------------------------------------------------------------------
//  The dispatch table, generated from include/console_commands.h.
// ---------------------------------------------------------------------------
struct Command {
  char        key;
  char        sub;      // 0 = single-letter command
  uint8_t     group;
  const char* help;
  Handler     fn;
};

const Command COMMANDS[] = {
#define CONSOLE_CMD(key, sub, group, help, handler) { key, sub, group, help, handler },
#include "console_commands.h"
#undef CONSOLE_CMD
};
constexpr size_t COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

// Two passes, in this order:
//   1. exact key+sub  -> two-letter commands ("KP0.5"), argument at line+2
//   2. key + sub == 0 -> single-letter catch-all ("V5", "K", "K5"), at line+1
// The catch-all is what preserves the old behaviour where an unrecognised
// second letter still reached the family's default branch.
void dispatch(const char* line) {
  char key = (char)toupper((unsigned char)line[0]);
  char sub = line[1] ? (char)toupper((unsigned char)line[1]) : 0;

  if (sub) {
    for (size_t i = 0; i < COMMAND_COUNT; i++) {
      if (COMMANDS[i].key == key && COMMANDS[i].sub == sub) {
        COMMANDS[i].fn((float)atof(line + 2));
        return;
      }
    }
  }
  for (size_t i = 0; i < COMMAND_COUNT; i++) {
    if (COMMANDS[i].key == key && COMMANDS[i].sub == 0) {
      COMMANDS[i].fn((float)atof(line + 1));
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

} // namespace

// ---------------------------------------------------------------------------
void printBanner() {
  printGroup(GRP_CMDS,   "Serial cmds: ");
  printGroup(GRP_GAINS,  "  gains:     ");
  printGroup(GRP_CONFIG, "  config:    ");
}

// ---------------------------------------------------------------------------
void update() {
  const char* line;
  while ((line = io::console::poll()) != nullptr) dispatch(line);

  Serial.print("t=");     Serial.print(millis());
  Serial.print(" #");     Serial.print(s_beat++);
  Serial.print(" mode="); Serial.print(state::axis.control_mode);

  // Telemetry channels, generated from the schema shared with the web GUI
  // (include/telemetry_schema.h). Add a channel THERE and it streams here and
  // appears in the GUI's graphs automatically. The status/CAN fields below are
  // not plotted channels, so they stay hand-written.
  // blnd touches the sensorless observer -> hall builds only.
#define TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr) \
  Serial.print(" " #key "="); Serial.print((float)(expr), prec);
#if SENSOR_TYPE == SENSOR_TYPE_HALL
#define TELEMETRY_CHANNEL_HALL TELEMETRY_CHANNEL
#else
#define TELEMETRY_CHANNEL_HALL(key, label, color, altkey, prec, expr)
#endif
#include "telemetry_schema.h"
#undef TELEMETRY_CHANNEL_HALL
#undef TELEMETRY_CHANNEL

  Serial.print(state::control.foc_ready ? " RUN" : (state::control.calibrated ? " idle" : " SAFE"));
  Serial.print(state::safety.fault ? " [FAULT]" : "");
  if (state::axis.axis_error) {
    Serial.print(" err=0x"); Serial.print(state::axis.axis_error, HEX);
  }
  Serial.print(" can_tx_ok=");   Serial.print(io::can::bus.txOkCount());
  Serial.print(" can_tx_fail="); Serial.print(io::can::bus.txFailCount());
  Serial.print(" can_rx=");      Serial.println(io::can::bus.rxCount());
}

}  // namespace console
}  // namespace app
