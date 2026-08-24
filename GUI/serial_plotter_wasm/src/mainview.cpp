#include "mainview.h"
#include "liveplot.h"
#include "serialbridge.h"
#include "telemetry.h"
#include "configparams.h"
#include "telemetryhub.h"

#include <QCheckBox>
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
#include <QSignalBlocker>
#include <QSplitter>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <array>
#include <utility>

namespace {
// ~1.5 h of telemetry at 10 Hz. Past this the oldest lines are dropped, so a
// session left running overnight can't exhaust the WASM heap.
constexpr size_t kMaxLogRows = 50000;
} // namespace

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
    connect(&hub, &TelemetryHub::logEvent, this, &MainView::onLogEvent);
    connect(&hub, &TelemetryHub::configReceived, this, &MainView::onConfigReceived);
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

    // CSV capture. Checkable so the button itself carries the state, with a
    // separate indicator line because a pressed-in button is easy to miss when
    // you come back to the bench.
    m_recordButton = new QPushButton(QStringLiteral("● Start recording"));
    m_recordButton->setCheckable(true);
    m_recordButton->setToolTip(
        QStringLiteral("Capture telemetry lines for CSV export. The graphs and "
                       "the serial monitor keep running either way."));
    m_recordIndicator = new QLabel;

    m_saveButton = new QPushButton(QStringLiteral("Save CSV"));
    ctrlLayout->addWidget(new QLabel(QStringLiteral("Time window (s)")), 0, 0);
    ctrlLayout->addWidget(m_windowSpin, 0, 1);
    ctrlLayout->addWidget(clearButton, 1, 0, 1, 2);
    ctrlLayout->addWidget(m_recordButton, 2, 0, 1, 2);
    ctrlLayout->addWidget(m_recordIndicator, 3, 0, 1, 2);
    ctrlLayout->addWidget(m_saveButton, 4, 0, 1, 2);

    // Monitor filter. Info is the default because that is the level at which
    // the log answers "what just happened" without answering "what is happening
    // every ten milliseconds" as well.
    m_logFilterCombo = new QComboBox;
    m_logFilterCombo->addItem(QStringLiteral("Errors only"));
    m_logFilterCombo->addItem(QStringLiteral("Warnings and above"));
    m_logFilterCombo->addItem(QStringLiteral("Info and above"));
    m_logFilterCombo->addItem(QStringLiteral("Everything"));
    m_logFilterCombo->setCurrentIndex(logevt::Info);
    m_logFilterCombo->setToolTip(QStringLiteral(
        "Hides lines in the monitor below. They are still received and still "
        "captured to CSV. To stop the board SENDING them, set its log level on "
        "the CAN Devices page."));
    ctrlLayout->addWidget(new QLabel(QStringLiteral("Monitor shows")), 5, 0);
    ctrlLayout->addWidget(m_logFilterCombo, 5, 1);

    m_recordTimer.setInterval(500);
    connect(&m_recordTimer, &QTimer::timeout, this, &MainView::updateRecordIndicator);
    updateRecordIndicator();

    auto *cmdGroup = new QGroupBox(QStringLiteral("Serial Commands"));
    auto *cmdLayout = new QVBoxLayout(cmdGroup);
    m_commandEdit = new QLineEdit;
    m_commandEdit->setPlaceholderText(
        QStringLiteral("e.g. A, I, C, T1.5, V10, KP0.1, X0.1, Q"));
    auto *sendButton = new QPushButton(QStringLiteral("Send"));
    auto *quickRow = new QHBoxLayout;
    const std::array<std::pair<const char *, const char *>, 3> quick = {{
        {"A (arm)", "A"}, {"I (idle)", "I"}, {"C (clear)", "C"}}};
    for (const auto &q : quick) {
        auto *b = new QPushButton(QString::fromLatin1(q.first));
        const QString cmd = QString::fromLatin1(q.second);
        connect(b, &QPushButton::clicked, this, [this, cmd] { sendCommand(cmd); });
        quickRow->addWidget(b);
    }
    cmdLayout->addWidget(m_commandEdit);
    cmdLayout->addWidget(sendButton);
    cmdLayout->addLayout(quickRow);

    // One checkbox per telemetry channel -> show/hide its chart in the shared
    // graph. Driven off kChannels so new channels (added in the shared schema)
    // appear here automatically.
    auto *chanGroup = new QGroupBox(QStringLiteral("Visible graphs"));
    auto *chanLayout = new QVBoxLayout(chanGroup);
    chanLayout->setSpacing(2);
    for (int ch = 0; ch < kNumChannels; ++ch) {
        auto *cb = new QCheckBox(QString::fromUtf8(kChannels[ch].label));
        cb->setChecked(true);
        cb->setStyleSheet(QStringLiteral("QCheckBox{color:%1;font-weight:bold;}")
                              .arg(QString::fromLatin1(kChannels[ch].color)));
        connect(cb, &QCheckBox::toggled, this,
                [this, ch](bool on) { m_plot->setChannelVisible(ch, on); });
        chanLayout->addWidget(cb);
    }

    layout->addWidget(ctrlGroup);
    layout->addWidget(cmdGroup);
    layout->addWidget(chanGroup);
    layout->addStretch(1);

    connect(clearButton, &QPushButton::clicked, this, &MainView::clearAll);
    connect(m_recordButton, &QPushButton::toggled, this, &MainView::onRecordToggled);
    connect(m_saveButton, &QPushButton::clicked, this, &MainView::onSaveCsv);
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

    // One PID group (KP/KI/KD spins + Apply) built once per loop from m_gains.
    // The decimals come from the shared config schema, so a spin never offers
    // more precision than the firmware will print back.
    QGroupBox *gainGroups[3];
    for (int g = 0; g < 3; ++g) {
        GainGroup &grp = m_gains[g];
        auto *box = new QGroupBox(QString::fromUtf8(grp.title));
        auto *grid = new QGridLayout(box);
        static const char *kRowLabel[3] = {"KP", "KI", "KD"};
        for (int i = 0; i < 3; ++i) {
            const ParamDef *def = configParam(grp.keys[i]);
            auto *spin = new QDoubleSpinBox;
            spin->setRange(0.0, 1000.0);
            spin->setDecimals(def ? def->decimals : 4);
            spin->setSingleStep(def && def->decimals >= 5 ? 0.001 : 0.01);
            grp.spin[i] = spin;
            grid->addWidget(new QLabel(QLatin1String(kRowLabel[i])), i, 0);
            grid->addWidget(spin, i, 1);
        }
        auto *applyBtn = new QPushButton(QStringLiteral("Apply"));
        grid->addWidget(applyBtn, 3, 0, 1, 2);
        connect(applyBtn, &QPushButton::clicked, this, [this, g] { applyGains(g); });
        gainGroups[g] = box;
    }

    // Read-back of the live config (serial Q): fills every spin above and shows
    // a summary, so you can see exactly what's on the board before tuning.
    auto *cfgGroup = new QGroupBox(QStringLiteral("Current configuration"));
    auto *cfgLayout = new QVBoxLayout(cfgGroup);
    auto *readCfgButton = new QPushButton(QStringLiteral("Read from board (Q)"));
    m_configSummary = new QLabel(
        QStringLiteral("Press “Read from board (Q)” to load the live values."));
    m_configSummary->setWordWrap(true);
    m_configSummary->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_configSummary->setStyleSheet(QStringLiteral("color:#555; font-size:11px;"));
    cfgLayout->addWidget(readCfgButton);
    cfgLayout->addWidget(m_configSummary);

    layout->addWidget(armGroup);
    layout->addWidget(spGroup);
    layout->addWidget(cfgGroup);
    for (QGroupBox *g : gainGroups)
        layout->addWidget(g);
    layout->addStretch(1);

    connect(armButton, &QPushButton::clicked, this, [this] { sendCommand(QStringLiteral("A")); });
    connect(idleButton, &QPushButton::clicked, this, [this] { sendCommand(QStringLiteral("I")); });
    connect(clearErrButton, &QPushButton::clicked, this, [this] { sendCommand(QStringLiteral("C")); });
    connect(clearGraphButton, &QPushButton::clicked, this, &MainView::clearAll);
    connect(applyButton, &QPushButton::clicked, this, &MainView::onApplySetpoint);
    connect(stopButton, &QPushButton::clicked, this, &MainView::onStopSetpoint);
    connect(readCfgButton, &QPushButton::clicked, this, &MainView::onReadConfig);

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

void MainView::appendLogRow(const QString &raw)
{
    if (!m_recording)
        return;
    m_logRows.push_back(raw);
    while (m_logRows.size() > kMaxLogRows)
        m_logRows.pop_front();
}

void MainView::onRecordToggled(bool on)
{
    m_recording = on;
    m_recordButton->setText(on ? QStringLiteral("■ Stop recording")
                               : QStringLiteral("● Start recording"));
    if (on) {
        // Deliberately NOT clearing here: pressing Stop then Start again
        // resumes the same capture rather than silently discarding it. Use
        // "Clear Graphs + Monitor" to start a genuinely fresh log.
        m_recordTimer.start();
        setStatus(m_logRows.empty()
                      ? QStringLiteral("Recording to CSV buffer")
                      : QStringLiteral("Recording resumed (%1 lines kept)")
                            .arg(m_logRows.size()));
    } else {
        m_recordTimer.stop();
        setStatus(QStringLiteral("Recording stopped — %1 lines captured")
                      .arg(m_logRows.size()));
    }
    updateRecordIndicator();
}

void MainView::updateRecordIndicator()
{
    if (!m_recordIndicator)
        return;
    const size_t n = m_logRows.size();
    if (m_recording) {
        m_recordIndicator->setText(
            QStringLiteral("● REC — %1 lines").arg(n));
        m_recordIndicator->setStyleSheet(
            QStringLiteral("color:#dc2626;font-weight:bold;"));
    } else {
        m_recordIndicator->setText(
            n == 0 ? QStringLiteral("○ Not recording")
                   : QStringLiteral("○ Stopped — %1 lines ready").arg(n));
        m_recordIndicator->setStyleSheet(QStringLiteral("color:#777;"));
    }
    if (m_saveButton)
        m_saveButton->setEnabled(n > 0);
}

void MainView::onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                           const QString &raw, const QHash<QString, double> &fields)
{
    Q_UNUSED(fields);   // re-parsed on export; see appendLogRow
    m_plot->addSample(tSec, vals);
    appendLogRow(raw);
}

void MainView::onLogEvent(const logevt::Event &e)
{
    // Recorded before the filter, always: a CSV that silently omits the lines
    // you had hidden is a trap, and the whole reason to capture one is to look
    // at it later with different questions in mind.
    appendLogRow(e.raw);

    if (int(e.level) > m_logFilterCombo->currentIndex())
        return;

    // Untagged lines (the board's own banner text) get no bracket, so the
    // firmware's boot output still reads the way it was written. The timestamp
    // is what turns a recurring fault into a measurable period.
    QString body = e.tag.isEmpty()
        ? e.text.toHtmlEscaped()
        : QStringLiteral("[%1] %2").arg(e.tag, e.text.toHtmlEscaped());
    const QString when = logevt::timeLabel(e);
    if (!when.isEmpty())
        body.prepend(QStringLiteral("%1  ").arg(when, 10));
    m_logView->appendHtml(QStringLiteral("<span style='color:%1'>%2</span>")
                              .arg(QLatin1String(logevt::levelColor(e.level)), body));
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

// Send one loop's KP/KI/KD, each with the command and precision the shared
// schema gives for that key. A gain with no write command is skipped rather
// than sent blind -- that is how a read-only row stays read-only here too.
void MainView::applyGains(int group)
{
    const GainGroup &grp = m_gains[group];
    for (int i = 0; i < 3; ++i) {
        const ParamDef *def = configParam(grp.keys[i]);
        if (!def || !def->cmdPrefix || !grp.spin[i])
            continue;
        sendCommand(QLatin1String(def->cmdPrefix)
                    + QString::number(grp.spin[i]->value(), 'f', def->decimals));
    }
}

void MainView::onReadConfig()
{
    sendCommand(QStringLiteral("Q"));
}

void MainView::onConfigReceived(const QHash<QString, double> &fields)
{
    // Fill each spin from its cfg field, leaving it alone when the key is
    // absent -- an older firmware that omits a key must not zero the box.
    // Signals blocked so populating does not read as a user edit.
    for (GainGroup &grp : m_gains) {
        for (int i = 0; i < 3; ++i) {
            const QString k = QLatin1String(grp.keys[i]);
            if (!grp.spin[i] || !fields.contains(k))
                continue;
            QSignalBlocker block(grp.spin[i]);
            grp.spin[i]->setValue(fields.value(k));
        }
    }

    if (!m_configSummary)
        return;

    // Summary lines: the two limits, then one line per PID loop. Values and
    // their precision come from the schema, so a parameter added there shows up
    // here without touching this function.
    auto show = [&](const char *key) {
        const ParamDef *def = configParam(key);
        const QString k = QLatin1String(key);
        return (def && fields.contains(k))
                   ? QString::number(fields.value(k), 'f', def->decimals)
                   : QStringLiteral("--");
    };

    QStringList lines;
    lines << QStringLiteral("Limits:  current %1 A   velocity %2 rad/s")
                 .arg(show("current_limit"), show("vel_limit"));
    for (const GainGroup &grp : m_gains) {
        lines << QStringLiteral("%1:  P %2   I %3   D %4")
                     .arg(QString::fromUtf8(grp.title).remove(QStringLiteral(" gains")),
                          show(grp.keys[0]), show(grp.keys[1]), show(grp.keys[2]));
    }
    m_configSummary->setText(lines.join(QLatin1Char('\n')));
    setStatus(QStringLiteral("Config loaded from board"));
}

void MainView::clearAll()
{
    m_plot->clear();
    m_logRows.clear();
    m_logView->clear();
    updateRecordIndicator();   // the captured-line count just went to zero
    setStatus(QStringLiteral("Cleared graphs and monitor"));
}

void MainView::onSaveCsv()
{
    // Columns follow the shared schema, so a channel added there is exported
    // without touching this function.
    QStringList headers = {QStringLiteral("raw_line"), QStringLiteral("t"),
                           QStringLiteral("mode")};
    for (int ch = 0; ch < kNumChannels; ++ch)
        headers << QString::fromLatin1(kChannels[ch].primaryKey);

    QString csv = headers.join(QLatin1Char(',')) + QLatin1Char('\n');
    for (const QString &raw : m_logRows) {
        // Parsing here rather than storing the fields per line: this runs once,
        // on demand, instead of on every line for the whole session.
        const QHash<QString, double> fields = telemetry::parseLine(raw);
        QString escaped = raw;
        escaped.replace(QLatin1Char('"'), QStringLiteral("\"\""));
        csv += QLatin1Char('"') + escaped + QLatin1Char('"');
        for (int i = 1; i < headers.size(); ++i) {
            csv += QLatin1Char(',');
            const QString key = headers.at(i);
            if (fields.contains(key))
                csv += QString::number(fields.value(key));
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
    setStatus(QStringLiteral("Saved %1 log lines%2")
                  .arg(m_logRows.size())
                  .arg(m_recording ? QStringLiteral(" (still recording)")
                                   : QString()));
}

void MainView::setStatus(const QString &text)
{
    m_statusLabel->setText(text);
}
