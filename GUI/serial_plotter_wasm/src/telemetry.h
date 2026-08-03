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

// Same tokenisation, but keeping the values as TEXT.
//
// Used for the "can ..." status line, whose fields are not all numbers: the
// error words are hex ("axis_err=0x140") and the seen-node bitmask is 64 bits,
// which does not survive a double. Keeping the raw token also lets the CAN
// Devices page show a value exactly as the board wrote it, which is what an
// operator comparing it against a datasheet actually wants.
inline QHash<QString, QString> parseTokens(const QString &line)
{
    QHash<QString, QString> fields;
    const int n = line.size();
    int i = 0;

    while (i < n) {
        while (i < n && (line[i] == QLatin1Char(' ') || line[i] == QLatin1Char('\t')))
            ++i;
        if (i >= n)
            break;

        const int keyStart = i;
        while (i < n) {
            const QChar c = line[i];
            if (c.isLetterOrNumber() || c == QLatin1Char('_'))
                ++i;
            else
                break;
        }
        if (i == keyStart) {
            ++i;
            continue;
        }
        const QString key = line.mid(keyStart, i - keyStart);

        if (i >= n || line[i] != QLatin1Char('=')) // bare word, e.g. the "can" prefix
            continue;
        ++i;

        const int valStart = i;
        while (i < n && line[i] != QLatin1Char(' ') && line[i] != QLatin1Char('\t'))
            ++i;
        if (i > valStart)
            fields.insert(key, line.mid(valStart, i - valStart));
    }

    return fields;
}

// Read one of those tokens as an integer, accepting both "42" and "0x2A".
// Returns `fallback` when the key is absent or unparseable, so a field an older
// firmware does not emit reads as its default rather than as zero.
inline quint64 tokenUInt(const QHash<QString, QString> &fields, const char *key,
                         quint64 fallback = 0)
{
    const QString v = fields.value(QString::fromLatin1(key));
    if (v.isEmpty())
        return fallback;
    bool ok = false;
    const quint64 parsed = v.startsWith(QStringLiteral("0x"), Qt::CaseInsensitive)
                               ? v.mid(2).toULongLong(&ok, 16)
                               : v.toULongLong(&ok, 10);
    return ok ? parsed : fallback;
}

} // namespace telemetry
