#include "pidtunerpage.h"
#include "liveplot.h"
#include "serialbridge.h"
#include "telemetryhub.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QVBoxLayout>

PidTunerPage::PidTunerPage(double windowS, QWidget *parent) : AppPage(parent)
{
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto *splitter = new QSplitter(Qt::Horizontal);
    layout->addWidget(splitter);

    m_sideScroll = new QScrollArea;
    m_sideScroll->setWidgetResizable(true);
    m_sideScroll->setFrameShape(QFrame::NoFrame);
    m_sideScroll->setWidget(buildSidePanel());

    // Right side: plot on top, log below.
    auto *rightSplit = new QSplitter(Qt::Vertical);
    m_plot = new LivePlot;
    m_plot->setWindow(windowS);
    m_logView = new QPlainTextEdit;
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(500);
    m_logView->setMinimumHeight(120);
    rightSplit->addWidget(m_plot);
    rightSplit->addWidget(m_logView);
    rightSplit->setStretchFactor(0, 1);
    rightSplit->setStretchFactor(1, 0);
    rightSplit->setSizes({640, 180});

    splitter->addWidget(m_sideScroll);
    splitter->addWidget(rightSplit);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({340, 900});

    auto &hub = TelemetryHub::instance();
    connect(&hub, &TelemetryHub::telemetry, this, &PidTunerPage::onTelemetry);
    connect(&hub, &TelemetryHub::message, this, &PidTunerPage::onMessage);
}

QWidget *PidTunerPage::buildSidePanel()
{
    auto *panel = new QWidget;
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    // Arming -------------------------------------------------------------
    auto *armGroup = new QGroupBox(QStringLiteral("Arming"));
    auto *armLayout = new QHBoxLayout(armGroup);
    auto *armButton = new QPushButton(QStringLiteral("Arm (A)"));
    auto *idleButton = new QPushButton(QStringLiteral("Idle (I)"));
    auto *clearButton = new QPushButton(QStringLiteral("Clear (C)"));
    armLayout->addWidget(armButton);
    armLayout->addWidget(idleButton);
    armLayout->addWidget(clearButton);

    // Setpoint -----------------------------------------------------------
    auto *spGroup = new QGroupBox(QStringLiteral("Setpoint"));
    auto *spLayout = new QGridLayout(spGroup);
    m_modeCombo = new QComboBox;
    m_modeCombo->addItem(QStringLiteral("Velocity [rad/s]"));
    m_modeCombo->addItem(QStringLiteral("Torque [Nm]"));
    m_modeCombo->addItem(QStringLiteral("Position [rad]"));
    m_setpointSpin = new QDoubleSpinBox;
    m_setpointSpin->setRange(-1000.0, 1000.0);
    m_setpointSpin->setDecimals(3);
    m_setpointSpin->setSingleStep(0.5);
    auto *applyButton = new QPushButton(QStringLiteral("Apply setpoint"));
    auto *stopButton = new QPushButton(QStringLiteral("Stop (0)"));
    spLayout->addWidget(new QLabel(QStringLiteral("Mode")), 0, 0);
    spLayout->addWidget(m_modeCombo, 0, 1);
    spLayout->addWidget(new QLabel(QStringLiteral("Target")), 1, 0);
    spLayout->addWidget(m_setpointSpin, 1, 1);
    spLayout->addWidget(applyButton, 2, 0);
    spLayout->addWidget(stopButton, 2, 1);

    // Velocity PID -------------------------------------------------------
    auto *pidGroup = new QGroupBox(QStringLiteral("Velocity PID gains"));
    auto *pidLayout = new QGridLayout(pidGroup);
    auto makeGain = [](int decimals, double step) {
        auto *s = new QDoubleSpinBox;
        s->setRange(0.0, 1000.0);
        s->setDecimals(decimals);
        s->setSingleStep(step);
        return s;
    };
    m_kpSpin = makeGain(4, 0.01);
    m_kiSpin = makeGain(4, 0.01);
    m_kdSpin = makeGain(5, 0.001);
    auto *applyGainsButton = new QPushButton(QStringLiteral("Apply gains"));
    pidLayout->addWidget(new QLabel(QStringLiteral("KP")), 0, 0);
    pidLayout->addWidget(m_kpSpin, 0, 1);
    pidLayout->addWidget(new QLabel(QStringLiteral("KI")), 1, 0);
    pidLayout->addWidget(m_kiSpin, 1, 1);
    pidLayout->addWidget(new QLabel(QStringLiteral("KD")), 2, 0);
    pidLayout->addWidget(m_kdSpin, 2, 1);
    pidLayout->addWidget(applyGainsButton, 3, 0, 1, 2);
    auto *pidHint = new QLabel(QStringLiteral(
        "Tip: read current gains on the Motor Config page (Q)."));
    pidHint->setWordWrap(true);
    pidHint->setStyleSheet(QStringLiteral("color:#666; font-size:11px;"));
    pidLayout->addWidget(pidHint, 4, 0, 1, 2);

    // Status -------------------------------------------------------------
    auto *statusGroup = new QGroupBox(QStringLiteral("Status"));
    auto *statusLayout = new QVBoxLayout(statusGroup);
    m_statusLabel = new QLabel(QStringLiteral("Idle"));
    m_statusLabel->setWordWrap(true);
    statusLayout->addWidget(m_statusLabel);

    layout->addWidget(armGroup);
    layout->addWidget(spGroup);
    layout->addWidget(pidGroup);
    layout->addWidget(statusGroup);
    layout->addStretch(1);
    panel->setMinimumWidth(320);

    connect(armButton, &QPushButton::clicked, this, [this] { send(QStringLiteral("A")); });
    connect(idleButton, &QPushButton::clicked, this, [this] { send(QStringLiteral("I")); });
    connect(clearButton, &QPushButton::clicked, this, [this] { send(QStringLiteral("C")); });
    connect(applyButton, &QPushButton::clicked, this, &PidTunerPage::onApplySetpoint);
    connect(stopButton, &QPushButton::clicked, this, &PidTunerPage::onStop);
    connect(applyGainsButton, &QPushButton::clicked, this, &PidTunerPage::onApplyGains);

    return panel;
}

void PidTunerPage::setSidePanelVisible(bool visible)
{
    m_sideScroll->setVisible(visible);
}

QString PidTunerPage::setpointPrefix() const
{
    switch (m_modeCombo->currentIndex()) {
    case 1: return QStringLiteral("T");
    case 2: return QStringLiteral("X");
    default: return QStringLiteral("V");
    }
}

void PidTunerPage::onApplySetpoint()
{
    send(setpointPrefix() + QString::number(m_setpointSpin->value(), 'f', 3));
}

void PidTunerPage::onStop()
{
    m_setpointSpin->setValue(0.0);
    send(setpointPrefix() + QStringLiteral("0"));
}

void PidTunerPage::onApplyGains()
{
    send(QStringLiteral("KP") + QString::number(m_kpSpin->value(), 'f', 4));
    send(QStringLiteral("KI") + QString::number(m_kiSpin->value(), 'f', 4));
    send(QStringLiteral("KD") + QString::number(m_kdSpin->value(), 'f', 5));
}

void PidTunerPage::send(const QString &command)
{
    auto &bridge = SerialBridge::instance();
    if (!bridge.isConnected()) {
        m_statusLabel->setText(QStringLiteral("Connect to the USB serial port first"));
        return;
    }
    bridge.writeLine(command);
    m_statusLabel->setText(QStringLiteral("Sent: %1").arg(command));
}

void PidTunerPage::onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                               const QString &, const QHash<QString, double> &)
{
    m_plot->addSample(tSec, vals);
}

void PidTunerPage::onMessage(const QString &raw)
{
    m_logView->appendPlainText(raw);
}
