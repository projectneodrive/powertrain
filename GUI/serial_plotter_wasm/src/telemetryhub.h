// Parses every serial line exactly once and fans the result out to whichever
// pages care. Without this, each page (two of which own a LivePlot) would
// re-parse the same stream.
//
// Three kinds of line, distinguished here:
//   - "cfg ..."  -> configReceived()  (the firmware's Q dump)
//   - "t=..."    -> telemetry()        (a sample for the plots)
//   - anything else -> message()       (firmware log/AK text)
#pragma once

#include <QHash>
#include <QObject>
#include <QString>
#include <array>

#include "channels.h"

class TelemetryHub : public QObject
{
    Q_OBJECT
public:
    static TelemetryHub &instance();

signals:
    // tSec is the firmware timestamp in seconds (absolute); each LivePlot
    // rebases it to its own t0. fields carries all parsed key=value pairs (for
    // CSV export); raw is the untouched line.
    void telemetry(double tSec, const std::array<double, kNumChannels> &vals,
                   const QString &raw, const QHash<QString, double> &fields);
    void message(const QString &raw);
    void configReceived(const QHash<QString, double> &fields);

private:
    TelemetryHub();
    void onLine(const QString &line);
};
