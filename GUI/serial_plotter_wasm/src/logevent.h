// The event-log line format, and its severities.
//
// The ESP32 control station (can_utilities/lib/logging) emits
//
//     log <sev> <tag> <free text>
//
// with <sev> in E/W/I/D and <tag> a short category (SYS CAN BUS LINK AXIS POT).
// That structure is what lets the monitor pane FILTER, which is the whole point:
// before it existed every line was equally important, so the pane scrolled past
// the two that mattered.
//
// Anything that does not match is still shown, as Info -- the board's own
// firmware log, its boot banner and every "AK ..." acknowledgement predate this
// format and are not going to change. They are classified here instead.
#pragma once

#include <QString>

namespace logevt {

enum Level {
    Error = 0,
    Warn  = 1,
    Info  = 2,
    Debug = 3,
};

struct Event {
    Level   level = Info;
    QString tag;      // "SYS", "AK", ... may be empty for unstructured lines
    QString text;     // message body, prefix stripped
    QString raw;      // the line exactly as received (what gets logged to CSV)
};

inline const char *levelName(Level l)
{
    switch (l) {
    case Error: return "Error";
    case Warn:  return "Warning";
    case Info:  return "Info";
    default:    return "Debug";
    }
}

// Monitor-pane colours. Chosen to stay legible on both the light and dark
// palettes Qt picks up from the browser; Debug is deliberately low-contrast so
// a trace does not visually outrank an error next to it.
inline const char *levelColor(Level l)
{
    switch (l) {
    case Error: return "#dc2626";
    case Warn:  return "#d97706";
    case Info:  return "#2563eb";
    default:    return "#6b7280";
    }
}

inline Level levelFromChar(QChar c)
{
    switch (c.toUpper().toLatin1()) {
    case 'E': return Error;
    case 'W': return Warn;
    case 'D': return Debug;
    default:  return Info;
    }
}

inline Event parse(const QString &line)
{
    Event e;
    e.raw = line;

    // "log <sev> <tag> <text>"
    if (line.startsWith(QStringLiteral("log "))) {
        const int sevPos = 4;
        if (line.size() > sevPos + 1 && line.at(sevPos + 1) == QLatin1Char(' ')) {
            e.level = levelFromChar(line.at(sevPos));
            const int tagStart = sevPos + 2;
            const int tagEnd = line.indexOf(QLatin1Char(' '), tagStart);
            if (tagEnd > tagStart) {
                e.tag = line.mid(tagStart, tagEnd - tagStart);
                e.text = line.mid(tagEnd + 1);
                return e;
            }
            // A tag with no text after it: still better than falling through.
            e.tag = line.mid(tagStart);
            return e;
        }
    }

    // "AK <cmd>: ..." -- a command acknowledgement from either the board or the
    // bridge. Always shown: these are the direct answer to something the
    // operator just did, which is the one class of line nobody wants filtered.
    if (line.startsWith(QStringLiteral("AK "))) {
        e.level = Info;
        e.tag = QStringLiteral("AK");
        e.text = line.mid(3);
        return e;
    }

    e.level = Info;
    e.text = line;
    return e;
}

} // namespace logevt
