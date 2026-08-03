// Axis state / control mode / error-bit names, for display.
//
// This list is NOT maintained here -- it is generated from the firmware-shared
// table (include/axis_vocab.h), the same one that generates the enums in
// gvl/axis_io.h and the decoder in can_utilities. Add an error bit there and it
// is named on this page, in the board's own log and in the bridge's, at once.
//
// See channels.h for the same arrangement applied to the telemetry channels.
#pragma once

#include <QString>
#include <QStringList>

namespace axisvocab {

inline QString state(unsigned v)
{
    switch (v) {
#define AXIS_STATE(name, value, label) case value: return QStringLiteral(label);
#define AXIS_MODE(name, value, label)
#define AXIS_ERROR(name, value, label)
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
    default: break;
    }
    return QStringLiteral("unknown (%1)").arg(v);
}

inline QString mode(unsigned v)
{
    switch (v) {
#define AXIS_STATE(name, value, label)
#define AXIS_MODE(name, value, label) case value: return QStringLiteral(label);
#define AXIS_ERROR(name, value, label)
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE
    default: break;
    }
    return QStringLiteral("unknown (%1)").arg(v);
}

// "MOTOR_FAILED | ENCODER_FAILED", or an empty string when there is no error.
// A bit this build has no name for is shown as raw hex rather than dropped: an
// unnamed bit is exactly the one worth surfacing, because it means the board is
// running a newer firmware than this page knows about.
inline QString errors(quint32 err)
{
    if (err == 0)
        return QString();

    QStringList parts;
    quint32 named = 0;
#define AXIS_STATE(name, value, label)
#define AXIS_MODE(name, value, label)
#define AXIS_ERROR(name, value, label)          \
    if (err & quint32(value)) {                 \
        parts << QStringLiteral(label);         \
        named |= quint32(value);                \
    }
#include "axis_vocab.h"
#undef AXIS_ERROR
#undef AXIS_MODE
#undef AXIS_STATE

    const quint32 unknown = err & ~named;
    if (unknown)
        parts << QStringLiteral("unnamed 0x%1").arg(unknown, 0, 16);
    return parts.join(QStringLiteral(" | "));
}

} // namespace axisvocab
