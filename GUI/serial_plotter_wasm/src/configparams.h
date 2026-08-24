// The configuration parameters the GUI reads from the board's "cfg ..." line.
// NOT maintained here -- generated from the firmware-shared schema so the GUI's
// table can never drift from what the firmware and the ESP32 station emit:
// add a parameter in include/config_schema.h and the row, its unit, its
// precision and its write command all appear here. See that file for the format.
#pragma once

#include <cstring>

struct ParamDef {
    const char *key;         // key in the firmware's cfg dump
    const char *label;
    const char *unit;
    const char *cmdPrefix;   // serial command that writes it, or nullptr = read-only
    int decimals;
    double demo;             // value the GUI's demo mode reports
};

// The firmware/station value expressions (last two schema columns) are unused
// here and discarded by the preprocessor, so they may name globals the GUI has
// never heard of. An empty cmd means no serial command writes the value, which
// is what makes the row read-only.
inline constexpr ParamDef kConfigParams[] = {
#define CONFIG_PARAM(key, label, unit, cmd, prec, demo, fw, br) \
    ParamDef{ #key, label, unit, (sizeof(cmd) > 1 ? cmd : nullptr), prec, demo },
#include "config_schema.h"
#undef CONFIG_PARAM
};

static constexpr int kNumConfigParams =
    int(sizeof(kConfigParams) / sizeof(kConfigParams[0]));

// Look a parameter up by its wire key. Returns nullptr for a key the firmware
// does not report, so a caller written against a newer schema degrades to
// "leave that field alone" rather than to a wrong precision.
inline const ParamDef *configParam(const char *key)
{
    for (const ParamDef &p : kConfigParams)
        if (std::strcmp(p.key, key) == 0)
            return &p;
    return nullptr;
}
