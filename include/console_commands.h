// Single source of truth for the serial console's command set.
//
// This file is an X-macro list, like telemetry_schema.h next to it. The
// includer #defines CONSOLE_CMD before #including it and #undefs it after.
// It is expanded TWICE in src/prog/prog_console.cpp: once to build the dispatch
// table, once to build the help banner printed at boot. Adding a command is one
// line here plus one handler function — the banner can no longer drift from the
// commands that actually exist, which it used to, being hand-written.
//
// SECOND CONSUMER: the ESP32 control station (can_utilities/) builds ITS serial
// console from this same table, so a keystroke means the same thing whether the
// GUI is plugged into the board's USB port or into the bridge. Adding a line
// here fails that build until the bridge says how the command maps onto
// CANSimple — or states that it does not (several gains are UART-only). The
// bridge's own extra commands live in can_utilities/lib/can_bridge/bridge_commands.h
// and must not reuse a key from this file.
//
//   CONSOLE_CMD(key, sub, group, help, handler)
//     key      first character of the command, UPPERCASE. Matching is
//              case-insensitive, so 'A' also accepts "a".
//     sub      second character, UPPERCASE, for two-letter commands such as
//              KP / LC / PD. Use 0 for a single-letter command.
//     group    which banner line it appears on: GRP_CMDS, GRP_GAINS, GRP_CONFIG
//     help     the banner fragment, or "" to keep the command out of the banner
//              (used for the I/D members of a family whose P line already
//              documents the whole set)
//     handler  void f(float v). v is the number parsed after the command:
//              atof(line+2) for a two-letter command, atof(line+1) otherwise.
//              Handlers of commands taking no argument simply ignore it.
//
// DISPATCH RULE: a key+sub exact match wins; failing that, an entry with the
// same key and sub == 0 catches everything else. That is what makes "KP0.5"
// reach cmdVelP while both "K" and "K5" reach cmdVelReapply.
//
// NB: a single-letter entry MUST be declared after its two-letter siblings for
// readability only — the dispatch does two passes, so the order in this file
// does not affect matching. It DOES affect the order of the banner.

// ---- Motion and state -------------------------------------------------------
CONSOLE_CMD('A', 0,   GRP_CMDS,   "A arm",          cmdArm)
CONSOLE_CMD('I', 0,   GRP_CMDS,   "I idle",         cmdIdle)
CONSOLE_CMD('V', 0,   GRP_CMDS,   "V<rad/s>",       cmdVelocity)
CONSOLE_CMD('T', 0,   GRP_CMDS,   "T<Nm>",          cmdTorque)
CONSOLE_CMD('X', 0,   GRP_CMDS,   "X<rad> pos",     cmdPosition)
CONSOLE_CMD('M', 0,   GRP_CMDS,   "M charac R/L",   cmdCharacterise)
#if SENSOR_TYPE == SENSOR_TYPE_HALL
CONSOLE_CMD('H', 0,   GRP_CMDS,   "H hall-cal",     cmdHallCal)
#endif
CONSOLE_CMD('C', 0,   GRP_CMDS,   "C clear",        cmdClearErrors)

// ---- Velocity PID gains (Nm/(rad/s)) ---------------------------------------
CONSOLE_CMD('K', 'P', GRP_GAINS,  "KP/KI/KD<v> vel",     cmdVelP)
CONSOLE_CMD('K', 'I', GRP_GAINS,  "",                    cmdVelI)
CONSOLE_CMD('K', 'D', GRP_GAINS,  "",                    cmdVelD)
CONSOLE_CMD('K', 0,   GRP_CMDS,   "KP/KI/KD<v> vel PID | K show", cmdVelReapply)

// ---- Current PID gains (V/A) ------------------------------------------------
CONSOLE_CMD('J', 'P', GRP_GAINS,  "JP/JI/JD<v> current", cmdCurP)
CONSOLE_CMD('J', 'I', GRP_GAINS,  "",                    cmdCurI)
CONSOLE_CMD('J', 'D', GRP_GAINS,  "",                    cmdCurD)
CONSOLE_CMD('J', 0,   GRP_GAINS,  "",                    cmdCurReapply)

// ---- Position PID gains -----------------------------------------------------
CONSOLE_CMD('P', 'P', GRP_GAINS,  "PP/PI/PD<v> position", cmdPosP)
CONSOLE_CMD('P', 'I', GRP_GAINS,  "",                     cmdPosI)
CONSOLE_CMD('P', 'D', GRP_GAINS,  "",                     cmdPosD)
CONSOLE_CMD('P', 0,   GRP_GAINS,  "",                     cmdPosReapply)

// ---- Runtime limits and configuration ---------------------------------------
CONSOLE_CMD('L', 'C', GRP_CONFIG, "LC<A> current-lim",    cmdLimitCurrent)
CONSOLE_CMD('L', 'V', GRP_CONFIG, "LV<rad/s> vel-lim",    cmdLimitVelocity)
CONSOLE_CMD('L', 0,   GRP_CONFIG, "",                     cmdLimitHelp)
CONSOLE_CMD('G', 0,   GRP_CONFIG, "G<v> pos-gain",        cmdPosGain)
CONSOLE_CMD('Q', 0,   GRP_CONFIG, "Q dump config (cfg ...)", cmdDumpConfig)
