// The telemetry channels the plotter displays. This list is NOT maintained
// here -- it is generated from the firmware-shared schema so the two can never
// drift: add a channel in include/telemetry_schema.h and it appears here (and
// on the board's serial stream) automatically. See that file for the format.
//
// TelemetryHub extracts these keys from each line; LivePlot draws one chart per
// channel.
#pragma once

struct ChannelDef {
    const char *label;
    const char *color;
    const char *primaryKey;
    const char *altKey;      // fallback key name, or nullptr
};

// Both schema macros expand to a ChannelDef for the GUI: hall-only channels are
// still listed so their chart/checkbox exists regardless of the board's build
// (they simply read 0 when the firmware doesn't send them). The firmware value
// expression (last macro arg) is unused here and discarded by the preprocessor.
inline constexpr ChannelDef kChannels[] = {
#define TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr) \
    ChannelDef{ label, color, #key, (sizeof(altkey) > 1 ? altkey : nullptr) },
#define TELEMETRY_CHANNEL_HALL TELEMETRY_CHANNEL
#include "telemetry_schema.h"
#undef TELEMETRY_CHANNEL_HALL
#undef TELEMETRY_CHANNEL
};

static constexpr int kNumChannels = int(sizeof(kChannels) / sizeof(kChannels[0]));
