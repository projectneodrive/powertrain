// Parses every serial line exactly once and fans the result out to whichever
// pages care. Without this, each page (two of which own a LivePlot) would
// re-parse the same stream.
//
// Four kinds of line, distinguished here:
//   - "cfg ..." -> configReceived()  the firmware's Q dump
//   - "can ..." -> canStatus()       the bridge's CAN device/bus status (1 Hz)
//   - "t=..."   -> telemetry()       a sample for the plots (10 Hz)
//   - anything else -> logEvent()    classified by severity, see logevent.h
//
// The can/log split is deliberate and is what made the monitor pane readable:
// counters and link state are DATA and belong on a page, while only genuine
// events reach the log. Both used to arrive as prose in the same stream.
#pragma once

#include <QHash>
#include <QObject>
#include <QString>
#include <array>

#include "channels.h"
#include "logevent.h"

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

    // One classified log line. Everything that is not cfg/can/telemetry ends up
    // here, including unstructured firmware text (as Info).
    void logEvent(const logevt::Event &event);

    void configReceived(const QHash<QString, double> &fields);

    // Raw tokens of a "can ..." line -- values kept as text because the error
    // words are hex and the node bitmask is 64 bits. See telemetry::tokenUInt.
    void canStatus(const QHash<QString, QString> &fields);

private:
    TelemetryHub();
    void onLine(const QString &line);
};
