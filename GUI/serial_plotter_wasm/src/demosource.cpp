#include "demosource.h"
#include "serialbridge.h"

#include <QByteArray>
#include <QRandomGenerator>
#include <QString>

#include <algorithm>
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
    m_brakeOn = false;

    const QByteArray banner =
        "--- SimpleFOC + FreeRTOS + CANSimple (DEMO, synthetic data) ---\n";
    SerialBridge::instance().feedBytes(banner.constData(), banner.size());

    // A representative config dump so the Motor Config page populates in demo
    // mode (matches reportConfig() in src/main.cpp).
    const QByteArray cfg =
        "cfg current_limit=4.000 vel_limit=17.780 pos_gain=1.0000 pos_i=0.0000 "
        "pos_d=0.00000 vel_p=0.5040 vel_i=0.0504 vel_d=0.00000 cur_p=1.0000 "
        "cur_i=50.0000 cur_d=0.00000 pole_pairs=26 kv=8.20 "
        "kt=1.0085 phase_r=4.2093 phase_l=4890.65 vbus_nom=24.0 volt_limit=23.5\n";
    SerialBridge::instance().feedBytes(cfg.constData(), cfg.size());
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

    // Sensorless observer health (mirrors HybridSensor): obsΔV stays ~0 (a bit
    // noisier at low speed), blend ramps 0->1 across the 5..7 rad/s crossover.
    const double av    = std::abs(m_vel);
    const double blnd  = (av <= 5.0) ? 0.0 : (av >= 7.0 ? 1.0 : (av - 5.0) / 2.0);
    const double obsdV = noise(0.05) + 0.3 * std::exp(-av / 1.5) * std::sin(m_ms / 250.0);

    QString line = QStringLiteral("t=%1 #%2 mode=1 tgt=%3 Iq=%4 vel=%5 pos=%6 Vbus=%7 "
                                  "Irgn=%8 Ibrk=%9 brk=%10 obsdV=%11 blnd=%12 RUN\n")
                       .arg(m_ms)
                       .arg(m_beat++)
                       .arg(m_tgt, 0, 'f', 2)
                       .arg(m_iq, 0, 'f', 2)
                       .arg(m_vel, 0, 'f', 2)
                       .arg(m_pos, 0, 'f', 2)
                       .arg(vbus, 0, 'f', 1)
                       .arg(irgn, 0, 'f', 2)
                       .arg(ibrk, 0, 'f', 2)
                       .arg(brk, 0, 'f', 2)
                       .arg(obsdV, 0, 'f', 2)
                       .arg(blnd, 0, 'f', 2);

    // Occasional non-telemetry line, so the serial monitor pane gets exercised
    // too (the parser must route these to the log, not the plots).
    if (m_beat % 40 == 0)
        line += QStringLiteral("AK V: vel %1 -> %2 rad/s\n").arg(m_vel, 0, 'f', 2).arg(m_tgt, 0, 'f', 2);

    const QByteArray utf8 = line.toUtf8();
    SerialBridge::instance().feedBytes(utf8.constData(), utf8.size());

    m_ms += kPeriodMs;
}
