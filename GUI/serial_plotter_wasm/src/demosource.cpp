#include "demosource.h"
#include "serialbridge.h"

#include <QByteArray>
#include <QRandomGenerator>
#include <QString>

#include <cmath>

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

    const QByteArray banner =
        "--- SimpleFOC + FreeRTOS + CANSimple (DEMO, synthetic data) ---\n";
    SerialBridge::instance().feedBytes(banner.constData(), banner.size());
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

    const double vbus = 24.0 + 0.4 * std::sin(m_ms / 900.0) + noise(0.05);

    QString line = QStringLiteral("t=%1 #%2 mode=1 tgt=%3 Iq=%4 vel=%5 pos=%6 Vbus=%7 RUN\n")
                       .arg(m_ms)
                       .arg(m_beat++)
                       .arg(m_tgt, 0, 'f', 2)
                       .arg(m_iq, 0, 'f', 2)
                       .arg(m_vel, 0, 'f', 2)
                       .arg(m_pos, 0, 'f', 2)
                       .arg(vbus, 0, 'f', 1);

    // Occasional non-telemetry line, so the serial monitor pane gets exercised
    // too (the parser must route these to the log, not the plots).
    if (m_beat % 40 == 0)
        line += QStringLiteral("AK V: vel %1 -> %2 rad/s\n").arg(m_vel, 0, 'f', 2).arg(m_tgt, 0, 'f', 2);

    const QByteArray utf8 = line.toUtf8();
    SerialBridge::instance().feedBytes(utf8.constData(), utf8.size());

    m_ms += kPeriodMs;
}
