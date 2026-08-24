// Single source of truth for the "cfg key=value ..." configuration line.
//
// Like telemetry_schema.h next to it, this is an X-macro list shared by every
// party that speaks the line:
//   * the firmware        (src/app/console.cpp, command 'Q')  -- emits it
//   * the ESP32 station   (can_utilities/.../bridge_console.cpp) -- emits its
//                          own version, from what it last COMMANDED
//   * the web GUI         (GUI/.../configparams.h)            -- table + tuner
//   * the GUI's demo mode (GUI/.../demosource.cpp)            -- synthesises it
//
// Before this table those five lived apart and drifted: the station printed
// fields the GUI's table did not show, and the GUI's table listed a precision
// the firmware did not use. Add a parameter HERE and every consumer picks it
// up; the two emitters fail to compile until each says where its value comes
// from, which is the point.
//
//   CONFIG_PARAM(key, label, unit, cmd, prec, demo, fw, br)
//     key    wire token, e.g. vel_p -> "vel_p=0.1000". Bare identifier.
//     label  human name for the GUI's table row          (string literal)
//     unit   unit shown beside it, "" for none           (string literal)
//     cmd    serial command that WRITES it ("KP" -> "KP0.1"), or "" if the
//            value is a compile-time constant. "" makes the GUI row read-only.
//     prec   decimal places, on the wire AND in the GUI   (int)
//     demo   plausible value for the GUI's demo mode      (double literal)
//     fw     firmware-side expression for the value. Only the firmware
//            compiles it. Constraint: no commas.
//     br     the same value as the ESP32 station knows it. Only the station
//            compiles it. Settable fields read back what the station last
//            commanded (CANSimple has no configuration read-back); the rest
//            are the CFG_* constants the firmware was built with.
//
// The GUI compiles neither expression -- its macro discards both -- so they may
// reference firmware/station globals freely.

// ---- Limits (writable) ------------------------------------------------------
CONFIG_PARAM(current_limit, "Current limit",      "A",            "LC", 2,  4.0,
             motor.current_limit,          c.current_limit)
CONFIG_PARAM(vel_limit,     "Velocity limit",     "rad/s",        "LV", 2, 17.78,
             motor.velocity_limit,         c.vel_limit)

// ---- Position loop ----------------------------------------------------------
CONFIG_PARAM(pos_gain,      "Position P",         "(rad/s)/rad",  "G",  4,  1.0,
             motor.P_angle.P,              c.pos_gain)
CONFIG_PARAM(pos_i,         "Position I",         "",             "PI", 4,  0.0,
             motor.P_angle.I,              CFG_POS_I)
CONFIG_PARAM(pos_d,         "Position D",         "",             "PD", 5,  0.0,
             motor.P_angle.D,              CFG_POS_D)

// ---- Velocity loop ----------------------------------------------------------
CONFIG_PARAM(vel_p,         "Velocity P",         "A/(rad/s)",    "KP", 4,  0.1,
             state::axis.vel_gain,         c.vel_gain)
CONFIG_PARAM(vel_i,         "Velocity I",         "A/rad",        "KI", 4,  0.05,
             state::axis.vel_int_gain,     c.vel_int_gain)
CONFIG_PARAM(vel_d,         "Velocity D",         "A·s/(rad/s)",  "KD", 5,  0.001,
             state::axis.vel_d_gain,       CFG_VEL_D)

// ---- Current loop -----------------------------------------------------------
CONFIG_PARAM(cur_p,         "Current P",          "V/A",          "JP", 4,  1.0,
             motor.PID_current_q.P,        CFG_CUR_P)
CONFIG_PARAM(cur_i,         "Current I",          "V/(A·s)",      "JI", 4, 50.0,
             motor.PID_current_q.I,        CFG_CUR_I)
CONFIG_PARAM(cur_d,         "Current D",          "V·s/A",        "JD", 5,  0.0,
             motor.PID_current_q.D,        CFG_CUR_D)

// ---- Hardware constants (read-only: no serial command writes these) ---------
CONFIG_PARAM(pole_pairs,    "Pole pairs",         "",             "",   0, 26.0,
             CFG_POLE_PAIRS,               CFG_POLE_PAIRS)
CONFIG_PARAM(kv,            "Motor KV",           "rpm/V",        "",   2,  8.2,
             CFG_KV,                       CFG_KV)
CONFIG_PARAM(kt,            "Torque constant Kt", "Nm/A",         "",   4,  1.0085,
             CFG_KT,                       CFG_KT)
CONFIG_PARAM(phase_r,       "Phase resistance",   "ohm",          "",   4,  4.2093,
             motor.phase_resistance,       CFG_PHASE_R)
CONFIG_PARAM(phase_l,       "Phase inductance",   "µH",           "",   2, 4890.65,
             motor.phase_inductance * 1e6f, CFG_PHASE_L * 1e6f)
CONFIG_PARAM(vbus_nom,      "Vbus nominal",       "V",            "",   1, 24.0,
             CFG_VBUS_NOMINAL,             CFG_VBUS_NOMINAL)
CONFIG_PARAM(volt_limit,    "Voltage limit",      "V",            "",   1, 23.5,
             motor.voltage_limit,          CFG_VOLT_LIMIT)
