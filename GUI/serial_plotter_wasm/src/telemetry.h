// Fast "key=value key=value" tokeniser for every line the board and the ESP32
// station emit:
//   t=12345 #42 mode=1 tgt=10.00 Iq=1.23 vel=9.80 pos=3.14 Vbus=24.5 RUN
//   cfg current_limit=4.00 vel_limit=17.78 pole_pairs=26 ...
//   can node=0 link=1 axis_err=0x140 nodes=0x0000000000000001 ...
// Bare words (#42, RUN, [FAULT], the "cfg"/"can" prefix) carry no '=' and are
// skipped.
#pragma once

#include <QHash>
#include <QString>

namespace telemetry {

// The one tokeniser. Calls sink(key, value) for every "key=value" pair, both as
// QStringView into `line` -- so nothing is allocated for a token a caller
// decides to drop. Whitespace either side of the '=' is tolerated; the wire
// never has any, but a hand-typed line might.
//
// NB: the callback is NOT named `emit`. That is a Qt keyword macro expanding to
// nothing, so `emit(k, v)` silently becomes the comma expression `(k, v)` and
// the tokeniser reports nothing at all.
template <typename Sink>
inline void forEachToken(const QString &line, Sink &&sink)
{
    const int n = line.size();
    int i = 0;
    const auto skipSpace = [&] {
        while (i < n && (line[i] == QLatin1Char(' ') || line[i] == QLatin1Char('\t')))
            ++i;
    };

    while (i < n) {
        skipSpace();
        if (i >= n)
            break;

        const int keyStart = i;
        while (i < n && (line[i].isLetterOrNumber() || line[i] == QLatin1Char('_')))
            ++i;
        if (i == keyStart) {          // not a key character -- skip it and retry
            ++i;
            continue;
        }
        const int keyLen = i - keyStart;

        skipSpace();
        if (i >= n || line[i] != QLatin1Char('='))   // bare word, e.g. "#42", "RUN"
            continue;
        ++i;
        skipSpace();

        const int valStart = i;
        while (i < n && line[i] != QLatin1Char(' ') && line[i] != QLatin1Char('\t'))
            ++i;
        if (i > valStart)
            sink(QStringView(line).mid(keyStart, keyLen),
                 QStringView(line).mid(valStart, i - valStart));
    }
}

// Numeric view: non-numeric values are dropped. Returns an empty hash when the
// line holds no numeric field at all (e.g. a firmware log line), which is how
// TelemetryHub tells telemetry from prose.
inline QHash<QString, double> parseLine(const QString &line)
{
    QHash<QString, double> fields;
    forEachToken(line, [&](QStringView key, QStringView value) {
        bool ok = false;
        const double v = value.toDouble(&ok);
        if (ok)
            fields.insert(key.toString(), v);
    });
    return fields;
}

// Text view, for the "can ..." status line, whose fields are not all numbers:
// the error words are hex ("axis_err=0x140") and the seen-node bitmask is 64
// bits, which does not survive a double. Keeping the raw token also lets the
// CAN Devices page show a value exactly as the board wrote it, which is what an
// operator comparing it against a datasheet actually wants.
inline QHash<QString, QString> parseTokens(const QString &line)
{
    QHash<QString, QString> fields;
    forEachToken(line, [&](QStringView key, QStringView value) {
        fields.insert(key.toString(), value.toString());
    });
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
