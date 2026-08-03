#include "telemetryhub.h"
#include "serialbridge.h"
#include "telemetry.h"

#include <cmath>
#include <limits>

TelemetryHub &TelemetryHub::instance()
{
    static TelemetryHub hub;
    return hub;
}

TelemetryHub::TelemetryHub()
{
    connect(&SerialBridge::instance(), &SerialBridge::lineReceived,
            this, &TelemetryHub::onLine);
}

void TelemetryHub::onLine(const QString &line)
{
    // Config dump: "cfg key=val ...". Checked before the telemetry path because
    // it also lacks a 't' field.
    if (line.startsWith(QStringLiteral("cfg "))) {
        emit configReceived(telemetry::parseLine(line));
        return;
    }

    // CAN device/bus status from the ESP32 control station, once a second.
    // Routed to the CAN Devices page and NEVER to the monitor pane: it is a
    // table's worth of counters, and printing it as prose is what made the log
    // unusable.
    if (line.startsWith(QStringLiteral("can "))) {
        emit canStatus(telemetry::parseTokens(line));
        return;
    }

    const QHash<QString, double> fields = telemetry::parseLine(line);
    if (!fields.contains(QStringLiteral("t"))) {
        emit logEvent(logevt::parse(line));
        return;
    }

    std::array<double, kNumChannels> vals{};
    for (int ch = 0; ch < kNumChannels; ++ch) {
        const ChannelDef &c = kChannels[ch];
        double v = fields.value(QString::fromLatin1(c.primaryKey),
                                std::numeric_limits<double>::quiet_NaN());
        if (std::isnan(v) && c.altKey)
            v = fields.value(QString::fromLatin1(c.altKey), 0.0);
        if (std::isnan(v))
            v = 0.0;
        vals[ch] = v;
    }
    emit telemetry(fields.value(QStringLiteral("t")) / 1000.0, vals, line, fields);
}
