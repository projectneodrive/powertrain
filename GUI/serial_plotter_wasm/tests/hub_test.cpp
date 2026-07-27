// Headless check of the parse/fan-out path: bytes fed to SerialBridge must come
// back out of TelemetryHub split correctly into config vs telemetry vs message.
// Build/run via tests/CMakeLists.txt (desktop qt-cmake). Exit 0 = pass.
#include "serialbridge.h"
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
    int msgCount = 0;
    QString lastMsg;

    auto &hub = TelemetryHub::instance();
    QObject::connect(&hub, &TelemetryHub::configReceived,
                     [&](const QHash<QString, double> &f) { gotConfig = true; cfg = f; });
    QObject::connect(&hub, &TelemetryHub::telemetry,
                     [&](double, const std::array<double, kNumChannels> &v, const QString &,
                         const QHash<QString, double> &) { ++telCount; lastVals = v; });
    QObject::connect(&hub, &TelemetryHub::message,
                     [&](const QString &m) { ++msgCount; lastMsg = m; });

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

    std::printf("Message path:\n");
    check(msgCount == 1, "one non-telemetry message");
    check(lastMsg.startsWith(QStringLiteral("AK V")), "message is the AK line");

    std::printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
