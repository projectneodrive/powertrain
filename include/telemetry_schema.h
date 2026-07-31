// Single source of truth for the streamed telemetry channels.
//
// This file is shared by BOTH sides of the wire:
//   * the firmware  (src/main.cpp, SerialTask)         -- emits "key=value"
//   * the web GUI   (GUI/serial_plotter_wasm channels)  -- plots one chart each
//
// Add a channel HERE, once, and it starts streaming from the board AND shows up
// in the web GUI's graphs/checkboxes automatically -- no other edit required.
//
// It is an X-macro list: the includer #defines TELEMETRY_CHANNEL (and, for
// channels that only exist on some firmware builds, TELEMETRY_CHANNEL_HALL)
// before #including this file, then #undefs them afterwards.
//
//   TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr)
//     key     wire token, e.g. Iq  ->  streamed as "Iq=...". Also the GUI's
//             primary lookup key. Must be a bare identifier (no quotes/spaces).
//     label   human name shown on the GUI chart / checkbox   (string literal)
//     color   chart colour, "#rrggbb"                         (string literal)
//     altkey  GUI fallback wire token, or "" for none         (string literal)
//     prec    decimal places the firmware prints              (int)
//     expr    firmware-side float expression giving the value. The GUI never
//             compiles this -- it is discarded by the GUI's macro -- so it may
//             reference firmware globals freely. Constraint: no commas.
//
// TELEMETRY_CHANNEL_HALL is identical but only emitted on hall builds (its expr
// touches the sensorless observer, which only exists there). The GUI always
// lists it so the chart/checkbox is available regardless of the board's build.
//
// NB: the demo generator (GUI .../demosource.cpp) synthesises fake values for
// these same keys; if you add a channel, give it a demo value there too.

TELEMETRY_CHANNEL(tgt,   "Target",        "#f97316", "",     2, g_active_target)
TELEMETRY_CHANNEL(Iq,    "Iq [A]",        "#22c55e", "iq",   2, g_io.iq_measured)
TELEMETRY_CHANNEL(vel,   "Vel [rad/s]",   "#3b82f6", "",     2, g_io.vel_rev * TWO_PI)
TELEMETRY_CHANNEL(pos,   "Pos [rad]",     "#a855f7", "",     2, g_io.pos_rev * TWO_PI)
TELEMETRY_CHANNEL(Vbus,  "Vbus [V]",      "#ef4444", "vbus", 1, g_io.vbus)

// --- Bilan énergétique du freinage ------------------------------------------
// Irgn : courant que le MOTEUR renvoie dans le bus (g_io.ibus < 0 = génératrice).
//        Tracé en positif pour se comparer directement à Ibrk. C'est l'énergie
//        qu'il faut absorber quelque part.
// Ibrk : courant moyen tiré par la résistance de freinage. C'est là qu'elle part.
// brk  : duty du chopper — sert à régler CFG_BRAKE_GAIN (voir board_config.h).
//
// Irgn > Ibrk durablement => la capacité de bus se charge => Vbus monte : soit
// augmenter CFG_BRAKE_GAIN, soit la résistance ne suffit pas.
//
// /!\ Ibrk est CALCULÉ (duty * Vbus / R), pas mesuré : la carte n'a aucun shunt
// sur la branche AUX. Il indique le courant COMMANDÉ. Si le demi-pont AUX ne
// conduit pas, la courbe monte quand même — c'est Vbus qui dit la vérité.
TELEMETRY_CHANNEL(Irgn,  "Regen [A]",     "#06b6d4", "",     2,
                  (g_io.ibus < 0.0f ? -g_io.ibus : 0.0f))
TELEMETRY_CHANNEL(Ibrk,  "Brake [A]",     "#eab308", "",     2,
                  (brake::duty() * g_io.vbus / CFG_BRAKE_R))
TELEMETRY_CHANNEL(brk,   "Brake duty",    "#f59e0b", "",     2, brake::duty())

// Sensorless flux-observer health (hall build only): obsdV (v_obs - v_hall)
// should stay ~0; blnd is the observer fraction 0 (hall) .. 1 (sensorless).
TELEMETRY_CHANNEL_HALL(obsdV, "obsdV [rad/s]", "#14b8a6", "", 2, hybrid.obs_dv)
TELEMETRY_CHANNEL_HALL(blnd,  "blend",         "#ec4899", "", 2, hybrid.blend)
