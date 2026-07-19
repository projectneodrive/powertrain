// Fast "key=value key=value" telemetry parser, ported from the Python plotter.
// The powertrain firmware (src/main.cpp, SerialTask) emits lines such as:
//   t=12345 #42 mode=1 tgt=10.00 Iq=1.23 vel=9.80 pos=3.14 Vbus=24.5 RUN
// We only extract the numeric key=value tokens; bare words (#42, RUN, [FAULT])
// are ignored.
#pragma once

#include <QHash>
#include <QString>

namespace telemetry {

// Parse a single telemetry line into a {key -> value} map. Returns an empty
// hash when no numeric field is found (e.g. a firmware log line).
inline QHash<QString, double> parseLine(const QString &line)
{
    QHash<QString, double> fields;
    const int n = line.size();
    int i = 0;

    while (i < n) {
        // skip whitespace
        while (i < n && (line[i] == QLatin1Char(' ') || line[i] == QLatin1Char('\t')))
            ++i;
        if (i >= n)
            break;

        // read key [A-Za-z0-9_]
        const int keyStart = i;
        while (i < n) {
            const QChar c = line[i];
            if (c.isLetterOrNumber() || c == QLatin1Char('_'))
                ++i;
            else
                break;
        }
        if (i == keyStart) {          // not a key char -> skip one and retry
            ++i;
            continue;
        }
        const QString key = line.mid(keyStart, i - keyStart);

        // expect '='
        while (i < n && (line[i] == QLatin1Char(' ') || line[i] == QLatin1Char('\t')))
            ++i;
        if (i >= n || line[i] != QLatin1Char('=')) // bare word like "#42", "RUN"
            continue;
        ++i;

        // read value up to next whitespace
        while (i < n && (line[i] == QLatin1Char(' ') || line[i] == QLatin1Char('\t')))
            ++i;
        const int valStart = i;
        while (i < n && line[i] != QLatin1Char(' ') && line[i] != QLatin1Char('\t'))
            ++i;
        if (i == valStart)
            continue;

        bool ok = false;
        const double value = line.mid(valStart, i - valStart).toDouble(&ok);
        if (ok)
            fields.insert(key, value);
    }

    return fields;
}

} // namespace telemetry
