#include "mainview.h"
#include "liveplot.h"
#include "serialbridge.h"
#include "telemetryhub.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <array>
#include <utility>

MainView::MainView(double windowS, QWidget *parent) : QWidget(parent)
{
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto *splitter = new QSplitter(Qt::Horizontal);
    layout->addWidget(splitter);

    // ---- left column: switchable control panels + a shared status line -----
    m_leftColumn = new QWidget;
    auto *leftLayout = new QVBoxLayout(m_leftColumn);
    leftLayout->setContentsMargins(0, 0, 0, 0);

    m_leftStack = new QStackedWidget;
    m_leftStack->addWidget(buildPlotterPanel(windowS));   // index 0
    m_leftStack->addWidget(buildTunerPanel());            // index 1

    auto *leftScroll = new QScrollArea;
    leftScroll->setWidgetResizable(true);
    leftScroll->setFrameShape(QFrame::NoFrame);
    leftScroll->setWidget(m_leftStack);

    m_statusLabel = new QLabel(QStringLiteral("Idle"));
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setStyleSheet(QStringLiteral("color:#444;"));

    leftLayout->addWidget(leftScroll, 1);
    leftLayout->addWidget(m_statusLabel);
    m_leftColumn->setMinimumWidth(330);

    // ---- right side: shared plot over shared log ---------------------------
    auto *right = new QSplitter(Qt::Vertical);
    m_plot = new LivePlot;
    m_plot->setWindow(windowS);
    m_logView = new QPlainTextEdit;
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(1000);
    m_logView->setMinimumHeight(90);
    m_logView->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    m_logView->setWordWrapMode(QTextOption::WrapAnywhere);
    right->addWidget(m_plot);
    right->addWidget(m_logView);
    right->setStretchFactor(0, 1);
    right->setStretchFactor(1, 0);
    right->setSizes({720, 150});

    splitter->addWidget(m_leftColumn);
    splitter->addWidget(right);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({360, 960});

    auto &hub = TelemetryHub::instance();
    connect(&hub, &TelemetryHub::telemetry, this, &MainView::onTelemetry);
    connect(&hub, &TelemetryHub::message, this, &MainView::onMessage);
    connect(&SerialBridge::instance(), &SerialBridge::statusChanged,
            this, &MainView::onConnectionChanged);
}

QWidget *MainView::buildPlotterPanel(double windowS)
{
    auto *panel = new QWidget;
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    auto *ctrlGroup = new QGroupBox(QStringLiteral("Controls"));
    auto *ctrlLayout = new QGridLayout(ctrlGroup);
    m_windowSpin = new QDoubleSpinBox;
    m_windowSpin->setRange(1.0, 300.0);
    m_windowSpin->setSingleStep(1.0);
    m_windowSpin->setDecimals(1);
    m_windowSpin->setValue(windowS);
    auto *clearButton = new QPushButton(QStringLiteral("Clear Graphs + Monitor"));
    auto *saveButton = new QPushButton(QStringLiteral("Save CSV"));
    ctrlLayout->addWidget(new QLabel(QStringLiteral("Time window (s)")), 0, 0);
    ctrlLayout->addWidget(m_windowSpin, 0, 1);
    ctrlLayout->addWidget(clearButton, 1, 0, 1, 2);
    ctrlLayout->addWidget(saveButton, 2, 0, 1, 2);

    auto *cmdGroup = new QGroupBox(QStringLiteral("Serial Commands"));
    auto *cmdLayout = new QVBoxLayout(cmdGroup);
    m_commandEdit = new QLineEdit;
    m_commandEdit->setPlaceholderText(
        QStringLiteral("e.g. A, I, M, C, T1.5, V10, KP0.1, Q"));
    auto *sendButton = new QPushButton(QStringLiteral("Send"));
    auto *quickRow = new QHBoxLayout;
    const std::array<std::pair<const char *, const char *>, 4> quick = {{
        {"A (arm)", "A"}, {"I (idle)", "I"}, {"M (measure)", "M"}, {"C (clear)", "C"}}};
    for (const auto &q : quick) {
        auto *b = new QPushButton(QString::fromLatin1(q.first));
        const QString cmd = QString::fromLatin1(q.second);
        connect(b, &QPushButton::clicked, this, [this, cmd] { sendCommand(cmd); });
        quickRow->addWidget(b);
    }
    cmdLayout->addWidget(m_commandEdit);
    cmdLayout->addWidget(sendButton);
    cmdLayout->addLayout(quickRow);

    layout->addWidget(ctrlGroup);
    layout->addWidget(cmdGroup);
    layout->addStretch(1);

    connect(clearButton, &QPushButton::clicked, this, &MainView::clearAll);
    connect(saveButton, &QPushButton::clicked, this, &MainView::onSaveCsv);
    connect(sendButton, &QPushButton::clicked, this, &MainView::onSendClicked);
    connect(m_commandEdit, &QLineEdit::returnPressed, this, &MainView::onSendClicked);
    connect(m_windowSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double v) { m_plot->setWindow(v); });

    return panel;
}

QWidget *MainView::buildTunerPanel()
{
    auto *panel = new QWidget;
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    auto *armGroup = new QGroupBox(QStringLiteral("Arming"));
    auto *armLayout = new QGridLayout(armGroup);
    auto *armButton = new QPushButton(QStringLiteral("Arm (A)"));
    auto *idleButton = new QPushButton(QStringLiteral("Idle (I)"));
    auto *clearErrButton = new QPushButton(QStringLiteral("Clear errors (C)"));
    auto *clearGraphButton = new QPushButton(QStringLiteral("Clear graphs"));
    armLayout->addWidget(armButton, 0, 0);
    armLayout->addWidget(idleButton, 0, 1);
    armLayout->addWidget(clearErrButton, 1, 0);
    armLayout->addWidget(clearGraphButton, 1, 1);

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

    layout->addWidget(armGroup);
    layout->addWidget(spGroup);
    layout->addWidget(pidGroup);
    layout->addStretch(1);

    connect(armButton, &QPushButton::clicked, this, [this] { sendCommand(QStringLiteral("A")); });
    connect(idleButton, &QPushButton::clicked, this, [this] { sendCommand(QStringLiteral("I")); });
    connect(clearErrButton, &QPushButton::clicked, this, [this] { sendCommand(QStringLiteral("C")); });
    connect(clearGraphButton, &QPushButton::clicked, this, &MainView::clearAll);
    connect(applyButton, &QPushButton::clicked, this, &MainView::onApplySetpoint);
    connect(stopButton, &QPushButton::clicked, this, &MainView::onStopSetpoint);
    connect(applyGainsButton, &QPushButton::clicked, this, &MainView::onApplyGains);

    return panel;
}

void MainView::showPanel(Panel panel)
{
    m_leftStack->setCurrentIndex(panel);
}

void MainView::setSidePanelVisible(bool visible)
{
    m_leftColumn->setVisible(visible);
}

// ------------------------------------------------------------- telemetry --

void MainView::onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                           const QString &raw, const QHash<QString, double> &fields)
{
    m_plot->addSample(tSec, vals);
    m_logRows.push_back({raw, fields});
}

void MainView::onMessage(const QString &raw)
{
    m_logView->appendPlainText(raw);
    m_logRows.push_back({raw, {}});
}

void MainView::onConnectionChanged(bool connected, const QString &message)
{
    setStatus(message);
    // Fresh graphs on every new connection.
    if (connected && !m_wasConnected)
        clearAll();
    m_wasConnected = connected;
}

// --------------------------------------------------------------- actions --

void MainView::sendCommand(const QString &command)
{
    const QString text = command.trimmed();
    if (text.isEmpty())
        return;
    auto &bridge = SerialBridge::instance();
    if (!bridge.isConnected()) {
        setStatus(QStringLiteral("Connect to the USB serial port first"));
        return;
    }
    bridge.writeLine(text);
    setStatus(QStringLiteral("Sent: %1").arg(text));
}

void MainView::onSendClicked()
{
    sendCommand(m_commandEdit->text());
    m_commandEdit->clear();
}

QString MainView::setpointPrefix() const
{
    switch (m_modeCombo->currentIndex()) {
    case 1: return QStringLiteral("T");
    case 2: return QStringLiteral("X");
    default: return QStringLiteral("V");
    }
}

void MainView::onApplySetpoint()
{
    sendCommand(setpointPrefix() + QString::number(m_setpointSpin->value(), 'f', 3));
}

void MainView::onStopSetpoint()
{
    m_setpointSpin->setValue(0.0);
    sendCommand(setpointPrefix() + QStringLiteral("0"));
}

void MainView::onApplyGains()
{
    sendCommand(QStringLiteral("KP") + QString::number(m_kpSpin->value(), 'f', 4));
    sendCommand(QStringLiteral("KI") + QString::number(m_kiSpin->value(), 'f', 4));
    sendCommand(QStringLiteral("KD") + QString::number(m_kdSpin->value(), 'f', 5));
}

void MainView::clearAll()
{
    m_plot->clear();
    m_logRows.clear();
    m_logView->clear();
    setStatus(QStringLiteral("Cleared graphs and monitor"));
}

void MainView::onSaveCsv()
{
    const QStringList headers = {"raw_line", "t", "mode", "tgt", "Iq", "vel", "pos", "Vbus"};
    QString csv = headers.join(QLatin1Char(',')) + QLatin1Char('\n');
    for (const LogRow &row : m_logRows) {
        QString escaped = row.raw;
        escaped.replace(QLatin1Char('"'), QStringLiteral("\"\""));
        csv += QLatin1Char('"') + escaped + QLatin1Char('"');
        for (int i = 1; i < headers.size(); ++i) {
            csv += QLatin1Char(',');
            const QString key = headers.at(i);
            if (row.fields.contains(key))
                csv += QString::number(row.fields.value(key));
        }
        csv += QLatin1Char('\n');
    }

    const QByteArray bytes = csv.toUtf8();
#ifdef __EMSCRIPTEN__
    QFileDialog::saveFileContent(bytes, QStringLiteral("serial_logs.csv"));
#else
    const QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("Save serial logs as CSV"),
        QStringLiteral("serial_logs.csv"), QStringLiteral("CSV Files (*.csv)"));
    if (path.isEmpty())
        return;
    QFile f(path);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(bytes);
        f.close();
    }
#endif
    setStatus(QStringLiteral("Saved %1 log lines").arg(m_logRows.size()));
}

void MainView::setStatus(const QString &text)
{
    m_statusLabel->setText(text);
}
