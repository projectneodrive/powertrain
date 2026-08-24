#include "demosource.h"
#include "channels.h"
#include "configparams.h"
#include "serialbridge.h"

#include <QByteArray>
#include <QRandomGenerator>
#include <QString>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

// Match SerialTask in src/main.cpp: one telemetry line every 100 ms.
constexpr int kPeriodMs = 100;

// Velocity step target cycles through these (rad/s), like a bench test.
constexpr double kSteps[] = {0.0, 20.0, 40.0, 10.0, -20.0, 0.0};
constexpr int kNumSteps = int(sizeof(kSteps) / sizeof(kSteps[0]));
constexpr int kStepPeriodMs = 4000;

double noise(double amplitude)
{
    return (QRandomGenerator::global()->generateDouble() - 0.5) * 2.0 * amplitude;
}

} // namespace

DemoSource::DemoSource(QObject *parent) : QObject(parent)
{
    m_timer.setInterval(kPeriodMs);
    connect(&m_timer, &QTimer::timeout, this, &DemoSource::tick);
}

void DemoSource::start()
{
    if (m_timer.isActive())
        return;
    m_ms = 0;
    m_beat = 0;
    m_tgt = m_vel = m_pos = m_iq = 0.0;
    m_brakeOn = false;
    m_demoFaulted = false;

    const QByteArray banner =
        "--- SimpleFOC + FreeRTOS + CANSimple (DEMO, synthetic data) ---\n";
    SerialBridge::instance().feedBytes(banner.constData(), banner.size());

    // A representative config dump so the Motor Config page populates in demo
    // mode. Built from the shared schema's demo column, so a parameter added
    // there appears here without touching this file.
    QString cfg = QStringLiteral("cfg");
    for (const ParamDef &p : kConfigParams)
        cfg += QStringLiteral(" %1=%2").arg(QLatin1String(p.key))
                                       .arg(p.demo, 0, 'f', p.decimals);
    cfg += QLatin1Char('\n');
    const QByteArray cfgBytes = cfg.toUtf8();
    SerialBridge::instance().feedBytes(cfgBytes.constData(), cfgBytes.size());
    m_timer.start();
}

void DemoSource::stop()
{
    m_timer.stop();
}

void DemoSource::tick()
{
    const double dt = kPeriodMs / 1000.0;

    // Stepped velocity setpoint.
    m_tgt = kSteps[(m_ms / kStepPeriodMs) % kNumSteps];

    // First-order lag toward the target, so vel/pos/Iq look like a real axis.
    const double error = m_tgt - m_vel;
    m_vel += error * (1.0 - std::exp(-dt / 0.25)) + noise(0.15);
    m_pos += m_vel * dt;
    m_iq = error * 0.08 + noise(0.05);

    // Bilan de freinage. Le moteur ne renvoie du courant que quand le couple
    // s'oppose à la rotation (décélération) -- on reproduit ce signe ici.
    const double ibus = -error * 0.08 * m_vel / 20.0;
    const double irgn = (ibus < 0.0) ? -ibus : 0.0;

    // Le chopper suit la même loi que brake::update() : hystérésis autour de
    // CFG_BRAKE_VBUS_ON/OFF puis gain proportionnel, plafonné à MAX_DUTY.
    double vbus = 24.0 + 0.4 * std::sin(m_ms / 900.0) + noise(0.05);
    vbus += irgn * 1.2;                       // la régen pousse le bus
    constexpr double kBrakeOn = 24.6, kBrakeOff = 24.2, kGain = 0.1, kMaxDuty = 0.7;
    if (!m_brakeOn && vbus > kBrakeOn)       m_brakeOn = true;
    else if (m_brakeOn && vbus < kBrakeOff)  m_brakeOn = false;
    double brk = m_brakeOn ? (vbus - kBrakeOff) * kGain : 0.0;
    brk = std::max(0.0, std::min(brk, kMaxDuty));
    const double ibrk = brk * vbus / 2.0;     // CFG_BRAKE_R = 2 ohms

    // Sensorless observer (mirrors HybridSensor): the blend ramps 0->1 across
    // the 5..7 rad/s crossover, and obsdV is the observer's disagreement with
    // the hall. A healthy observer keeps obsdV inside HybridSensor's tolerance
    // (0.5 + 0.15*|vel|); the demo stays just inside it, so the chart shows what
    // "good" looks like when you compare it against a real capture.
    const double av    = std::abs(m_vel);
    const double blnd  = (av <= 5.0) ? 0.0 : (av >= 7.0 ? 1.0 : (av - 5.0) / 2.0);
    const double obsdV = noise(0.25) + 0.02 * m_vel;

    // The channel values, by wire key. The LINE itself is generated from the
    // shared schema below, so a channel added there always appears here -- as
    // zero until it is given a value in this table. That is the whole reason
    // demo mode stopped drifting from the firmware's stream.
    const struct { const char *key; double value; } kDemoValues[] = {
        {"tgt", m_tgt}, {"Iq", m_iq},   {"vel", m_vel},   {"pos", m_pos},
        {"Vbus", vbus}, {"Irgn", irgn}, {"Ibrk", ibrk},
        {"blnd", blnd}, {"obsdV", obsdV},
    };

    QString line = QStringLiteral("t=%1 #%2 mode=1").arg(m_ms).arg(m_beat++);
    for (int ch = 0; ch < kNumChannels; ++ch) {
        const ChannelDef &c = kChannels[ch];
        double v = 0.0;
        for (const auto &kv : kDemoValues) {
            if (std::strcmp(kv.key, c.primaryKey) == 0) { v = kv.value; break; }
        }
        line += QStringLiteral(" %1=%2").arg(QLatin1String(c.primaryKey))
                                        .arg(v, 0, 'f', c.decimals);
    }
    line += QStringLiteral(" RUN\n");

    // Occasional non-telemetry line, so the serial monitor pane gets exercised
    // too (the parser must route these to the log, not the plots).
    if (m_beat % 40 == 0)
        line += QStringLiteral("AK V: vel %1 -> %2 rad/s\n").arg(m_vel, 0, 'f', 2).arg(m_tgt, 0, 'f', 2);

    // The bridge's CAN status line, once a second, so the CAN Devices page is
    // populated in demo mode. Mirrors emitCanStatus() in
    // can_utilities/lib/can_bridge/bridge_telemetry.cpp -- keep the key set in
    // step or the page silently shows defaults.
    if (m_beat % 10 == 0) {
        // Fault the axis for a few seconds mid-cycle, so the error decoding and
        // the red tinting are visible without needing a broken board.
        const bool faulted = (m_ms / 1000) % 30 >= 25;
        const quint32 axisErr = faulted ? 0x140u : 0u;
        const int axisState = faulted ? 1 : 8;
        line += QStringLiteral(
                    "can node=0 link=1 hb_age=%1 hb_period=100 hb_timeout=500 "
                    "hb_max=104 drops=0 scan_max=2 stop_after=3000 stopped=0 bus=1 "
                    "axis=%2 mode=2 axis_err=0x%3 motor_err=0x%4 enc_err=0x0 ctrl_err=0x0 "
                    "tx_ok=%5 tx_fail=0 rx=%6 tx_ec=0 rx_ec=0 tx_failed=0 rx_missed=0 "
                    "rx_overrun=0 arb_lost=0 bus_ec=%7 baud=500000 "
                    "nodes=0x0000000000000001 loglvl=2\n")
                    .arg(m_ms % 100)
                    .arg(axisState)
                    .arg(axisErr, 0, 16)
                    .arg(faulted ? 1 : 0, 0, 16)
                    .arg(m_beat / 4)
                    .arg(m_beat * 4)
                    .arg(faulted ? 3 : 0);

        // ...and the matching event, on the transition only -- which is exactly
        // what the station does, and what makes the monitor readable.
        if (faulted != m_demoFaulted) {
            m_demoFaulted = faulted;
            line += faulted
                ? QStringLiteral("log E AXIS error 0x0 -> 0x140 [MOTOR_FAILED|ENCODER_FAILED]\n")
                : QStringLiteral("log I AXIS error cleared (was 0x140)\n");
        }
    }

    const QByteArray utf8 = line.toUtf8();
    SerialBridge::instance().feedBytes(utf8.constData(), utf8.size());

    m_ms += kPeriodMs;
}
