// Single source of truth for the streamed telemetry channels.
//
// This file is shared by all three parties on the wire:
//   * the firmware  (src/prog/prog_console.cpp)         -- emits "key=value"
//   * the web GUI   (GUI/serial_plotter_wasm channels)  -- plots one chart each
//   * the ESP32 CAN control station (can_utilities/)    -- re-emits the same
//     line from CAN telemetry, so the GUI can be pointed at either port
//
// Add a channel HERE, once, and it starts streaming from the board AND shows up
// in the web GUI's graphs/checkboxes automatically -- no other edit required.
// can_utilities is the one that will not silently ignore you: it needs one
// accessor per channel, so a new channel fails ITS link until somebody decides
// whether that value is reachable over CAN at all. That is deliberate.
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

TELEMETRY_CHANNEL(tgt,   "Target",        "#f97316", "",     2, gvl::Q.active_target)
TELEMETRY_CHANNEL(Iq,    "Iq [A]",        "#22c55e", "iq",   2, gvl::AXIS.iq_measured)
TELEMETRY_CHANNEL(vel,   "Vel [rad/s]",   "#3b82f6", "",     2, gvl::AXIS.vel_rev * TWO_PI)
TELEMETRY_CHANNEL(pos,   "Pos [rad]",     "#a855f7", "",     2, gvl::AXIS.pos_rev * TWO_PI)
TELEMETRY_CHANNEL(Vbus,  "Vbus [V]",      "#ef4444", "vbus", 1, gvl::AXIS.vbus)

// --- Braking energy balance --------------------------------------------------
// Irgn : current the MOTOR pushes back into the bus (ibus < 0 = generating).
//        Plotted positive so it compares directly against Ibrk. This is the
//        energy that has to be absorbed somewhere.
// Ibrk : mean current drawn by the brake resistor. This is where it goes.
//
// Irgn > Ibrk for any length of time => the bus capacitance charges => Vbus
// climbs: either raise CFG_BRAKE_GAIN, or the resistor is not enough.
//
// /!\ Ibrk is COMPUTED (duty * Vbus / R), not measured: the board has no shunt
// on the AUX branch. It shows the COMMANDED current. If the AUX half-bridge is
// not conducting, the curve rises anyway — Vbus is the one telling the truth.
TELEMETRY_CHANNEL(Irgn,  "Regen [A]",     "#06b6d4", "",     2,
                  (gvl::AXIS.ibus < 0.0f ? -gvl::AXIS.ibus : 0.0f))
TELEMETRY_CHANNEL(Ibrk,  "Brake [A]",     "#eab308", "",     2,
                  (io::brake::duty() * gvl::AXIS.vbus / CFG_BRAKE_R))

// Sensorless observer fraction (hall build only): 0 = pure hall, 1 = sensorless.
TELEMETRY_CHANNEL_HALL(blnd,  "blend",         "#ec4899", "", 2, io::motor::hybrid.blend)
