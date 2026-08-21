// Headless check of the parse/fan-out path: bytes fed to SerialBridge must come
// back out of TelemetryHub split correctly into config / can / telemetry / log,
// and log lines must be classified by severity.
// Build/run via tests/CMakeLists.txt (desktop qt-cmake). Exit 0 = pass.
#include "axisvocab.h"
#include "logevent.h"
#include "serialbridge.h"
#include "telemetry.h"
#include "telemetryhub.h"

#include <QCoreApplication>
#include <QHash>

#include <cstdio>
#include <cstring>

static int failures = 0;
static void check(bool cond, const char *what)
{
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond)
        ++failures;
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);

    bool gotConfig = false;
    QHash<QString, double> cfg;
    int telCount = 0;
    std::array<double, kNumChannels> lastVals{};
    int logCount = 0;
    logevt::Event lastLog;
    int canCount = 0;
    QHash<QString, QString> lastCan;

    auto &hub = TelemetryHub::instance();
    QObject::connect(&hub, &TelemetryHub::configReceived,
                     [&](const QHash<QString, double> &f) { gotConfig = true; cfg = f; });
    QObject::connect(&hub, &TelemetryHub::telemetry,
                     [&](double, const std::array<double, kNumChannels> &v, const QString &,
                         const QHash<QString, double> &) { ++telCount; lastVals = v; });
    QObject::connect(&hub, &TelemetryHub::logEvent,
                     [&](const logevt::Event &e) { ++logCount; lastLog = e; });
    QObject::connect(&hub, &TelemetryHub::canStatus,
                     [&](const QHash<QString, QString> &f) { ++canCount; lastCan = f; });

    auto feed = [](const char *s) {
        SerialBridge::instance().feedBytes(s, int(std::strlen(s)));
    };

    feed("cfg current_limit=4.000 vel_limit=17.780 pos_gain=1.0000 vel_p=0.5040 "
         "pole_pairs=26 kt=1.0085\n");
    feed("t=1000 #1 mode=1 tgt=5.00 Iq=1.20 vel=4.80 pos=3.10 Vbus=24.5 RUN\n");
    feed("AK V: vel 0.00 -> 5.00 rad/s\n");
    // A chunk split across two feeds must still parse as one line.
    feed("t=1100 #2 mode=1 tgt=5.00 Iq=1.10 vel=4.9");
    feed("0 pos=3.60 Vbus=24.4 RUN\n");

    std::printf("Config path:\n");
    check(gotConfig, "cfg line -> configReceived");
    check(std::abs(cfg.value(QStringLiteral("current_limit")) - 4.0) < 1e-9, "current_limit=4.0");
    check(std::abs(cfg.value(QStringLiteral("pole_pairs")) - 26.0) < 1e-9, "pole_pairs=26");
    check(!cfg.contains(QStringLiteral("t")), "cfg has no bogus 't' field");

    std::printf("Telemetry path:\n");
    check(telCount == 2, "two telemetry samples (incl. the split line)");
    check(std::abs(lastVals[2] - 4.9) < 1e-9, "vel channel of split line = 4.90");
    check(std::abs(lastVals[4] - 24.4) < 1e-9, "Vbus channel of split line = 24.4");

    std::printf("Log path:\n");
    check(logCount == 1, "one log event so far (the AK line)");
    check(lastLog.tag == QStringLiteral("AK"), "AK line tagged AK");
    check(lastLog.level == logevt::Info, "AK line is Info");

    feed("log 12345 E AXIS error 0x0 -> 0x140 [MOTOR_FAILED|ENCODER_FAILED]\n");
    check(lastLog.level == logevt::Error, "'log <ms> E ...' -> Error");
    check(lastLog.tag == QStringLiteral("AXIS"), "tag extracted");
    check(lastLog.text.startsWith(QStringLiteral("error 0x0")), "text has the prefix stripped");
    check(lastLog.hasTime && lastLog.tMs == 12345, "timestamp extracted");
    check(logevt::timeLabel(lastLog) == QStringLiteral("12.345"), "timestamp rendered as s.mmm");

    // A station flashed with an older build emits no timestamp. The two
    // firmwares are flashed separately, so this is a normal state to be in.
    feed("log W LINK lost with node 0\n");
    check(lastLog.level == logevt::Warn, "timestamp-less line still parses");
    check(lastLog.tag == QStringLiteral("LINK"), "...and still yields its tag");
    check(!lastLog.hasTime, "...and is marked as having no timestamp");

    feed("log 500 D CAN TX id=0x00D CMD_SET_INPUT_VEL len=8\n");
    check(lastLog.level == logevt::Debug, "'log <ms> D ...' -> Debug");

    feed("--- SimpleFOC + FreeRTOS + CANSimple ---\n");
    check(lastLog.level == logevt::Info, "unstructured firmware text -> Info");
    check(lastLog.tag.isEmpty(), "unstructured text has no tag");
    check(lastLog.text == QStringLiteral("--- SimpleFOC + FreeRTOS + CANSimple ---"),
          "unstructured text passed through verbatim");

    std::printf("CAN status path:\n");
    // Snapshot rather than a hardcoded total: the point of the last check is
    // that the can line adds NOTHING to the log, and a literal count silently
    // becomes a different assertion the moment anyone adds a feed above.
    const int logCountBeforeCan = logCount;
    feed("can node=3 link=1 hb_age=8 bus=1 axis=8 mode=2 axis_err=0x140 motor_err=0x0 "
         "tx_ok=1234 rx=5678 bus_ec=0 baud=500000 nodes=0x0000000000000009 loglvl=2\n");
    check(canCount == 1, "can line -> canStatus");
    check(telemetry::tokenUInt(lastCan, "node") == 3, "node=3 (decimal)");
    check(telemetry::tokenUInt(lastCan, "axis_err") == 0x140, "axis_err=0x140 (hex)");
    check(telemetry::tokenUInt(lastCan, "nodes") == 0x9, "64-bit node mask survives");
    check(telemetry::tokenUInt(lastCan, "absent", 42) == 42, "missing key -> fallback");
    check(logCount == logCountBeforeCan, "the can line did NOT reach the log");

    std::printf("Shared axis vocabulary (include/axis_vocab.h):\n");
    check(axisvocab::state(8) == QStringLiteral("CLOSED_LOOP"), "state 8 = CLOSED_LOOP");
    check(axisvocab::mode(2) == QStringLiteral("VELOCITY"), "mode 2 = VELOCITY");
    check(axisvocab::errors(0x140) == QStringLiteral("MOTOR_FAILED | ENCODER_FAILED"),
          "0x140 decodes to two named bits");
    check(axisvocab::errors(0).isEmpty(), "0 decodes to nothing");
    check(axisvocab::errors(0x80000000).contains(QStringLiteral("unnamed")),
          "an unnamed bit is surfaced, not dropped");

    std::printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
