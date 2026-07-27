// The five telemetry channels the plotter displays, shared by TelemetryHub
// (which extracts them from each line) and LivePlot (which draws them).
// Field names match what SerialTask emits in src/main.cpp.
#pragma once

static constexpr int kNumChannels = 5;

struct ChannelDef {
    const char *label;
    const char *color;
    const char *primaryKey;
    const char *altKey;      // fallback key name, or nullptr
};

inline constexpr ChannelDef kChannels[kNumChannels] = {
    {"Target",      "#f97316", "tgt",  nullptr},
    {"Iq [A]",      "#22c55e", "Iq",   "iq"},
    {"Vel [rad/s]", "#3b82f6", "vel",  nullptr},
    {"Pos [rad]",   "#a855f7", "pos",  nullptr},
    {"Vbus [V]",    "#ef4444", "Vbus", "vbus"},
};
