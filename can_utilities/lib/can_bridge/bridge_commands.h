// Commands that exist only on the control station.
//
// An X-macro list in the same shape as the firmware's include/console_commands.h,
// which this station also compiles: the two are concatenated into one dispatch
// table in bridge_console.cpp, so a keystroke that means something on the board
// keeps meaning it here, and these are the extras on top.
//
//   BRIDGE_CMD(key, sub, group, help, handler)   — fields as in console_commands.h
//
// A key here MUST NOT appear in console_commands.h. Nothing stops you at
// compile time (the tables are built independently), so bridge_console.cpp
// checks at boot and complains loudly on the console instead. The free letters
// at the time of writing were B D E F N O R S U W Y Z.
//
// Two kinds of command live here, and it is worth knowing which is which:
//
//   * things only the BRIDGE can do — it owns the potentiometer and the CAN
//     trace, neither of which the board knows exists;
//   * things CANSimple exposes but the board's own serial console does not —
//     e-stop and reboot are frames (CMD_ESTOP, CMD_REBOOT) with no console
//     equivalent, because on the board itself you can just unplug it.

// ---- Axis commands reachable over CAN but absent from the board's console ---
BRIDGE_CMD('E', 0, GRP_CMDS,   "E estop",              cmdEstop)
BRIDGE_CMD('R', 0, GRP_CMDS,   "R reboot board",       cmdReboot)
BRIDGE_CMD('F', 0, GRP_CMDS,   "F fault codes",        cmdFaults)

// ---- The station's own controls --------------------------------------------
// D sets the event-log level: 0 errors, 1 +warnings, 2 +state changes (default),
// 3 +per-frame CAN trace. It is a LEVEL rather than the old on/off frame-trace
// toggle because "too much log" has more than two useful answers — the trace is
// only one of the things that was drowning the monitor pane.
BRIDGE_CMD('D', 0, GRP_BRIDGE, "D<0-3> log level",     cmdLogLevel)
BRIDGE_CMD('Z', 0, GRP_BRIDGE, "Z pot rest cal",       cmdPotRest)
BRIDGE_CMD('?', 0, GRP_BRIDGE, "? help",               cmdHelp)
