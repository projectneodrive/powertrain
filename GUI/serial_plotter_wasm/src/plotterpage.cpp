#include "plotterpage.h"
#include "liveplot.h"
#include "serialbridge.h"
#include "telemetryhub.h"

#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QPlainTextEdit>
#include <QSplitter>
#include <QVBoxLayout>

#include <array>
#include <utility>

PlotterPage::PlotterPage(double windowS, QWidget *parent) : AppPage(parent)
{
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto *splitter = new QSplitter(Qt::Horizontal);
    layout->addWidget(splitter);

    m_sideScroll = new QScrollArea;
    m_sideScroll->setWidgetResizable(true);
    m_sideScroll->setFrameShape(QFrame::NoFrame);
    m_sideScroll->setWidget(buildSidePanel(windowS));

    m_plot = new LivePlot;
    m_plot->setWindow(windowS);

    splitter->addWidget(m_sideScroll);
    splitter->addWidget(m_plot);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({360, 900});

    auto &hub = TelemetryHub::instance();
    connect(&hub, &TelemetryHub::telemetry, this, &PlotterPage::onTelemetry);
    connect(&hub, &TelemetryHub::message, this, &PlotterPage::onMessage);
}

QWidget *PlotterPage::buildSidePanel(double windowS)
{
    auto *panel = new QWidget;
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    // Controls -----------------------------------------------------------
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

    // Serial commands ----------------------------------------------------
    auto *cmdGroup = new QGroupBox(QStringLiteral("Serial Commands"));
    auto *cmdLayout = new QVBoxLayout(cmdGroup);
    m_commandEdit = new QLineEdit;
    m_commandEdit->setPlaceholderText(
        QStringLiteral("Type a command, e.g. A, I, M, C, T1.5, V10, KP0.1"));
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

    // Status -------------------------------------------------------------
    auto *statusGroup = new QGroupBox(QStringLiteral("Status"));
    auto *statusLayout = new QVBoxLayout(statusGroup);
    m_statusLabel = new QLabel(QStringLiteral("Idle"));
    m_statusLabel->setWordWrap(true);
    statusLayout->addWidget(m_statusLabel);

    // Logs ---------------------------------------------------------------
    auto *logsGroup = new QGroupBox(QStringLiteral("Serial Logs"));
    auto *logsLayout = new QVBoxLayout(logsGroup);
    m_logView = new QPlainTextEdit;
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(1000);
    m_logView->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    m_logView->setWordWrapMode(QTextOption::WrapAnywhere);
    m_logView->setMinimumHeight(200);
    logsLayout->addWidget(m_logView);

    layout->addWidget(ctrlGroup);
    layout->addWidget(cmdGroup);
    layout->addWidget(statusGroup);
    layout->addWidget(logsGroup);
    layout->addStretch(1);
    panel->setMinimumWidth(340);

    connect(clearButton, &QPushButton::clicked, this, &PlotterPage::onClearClicked);
    connect(saveButton, &QPushButton::clicked, this, &PlotterPage::onSaveCsv);
    connect(sendButton, &QPushButton::clicked, this, &PlotterPage::onSendClicked);
    connect(m_commandEdit, &QLineEdit::returnPressed, this, &PlotterPage::onSendClicked);
    connect(m_windowSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double v) { m_plot->setWindow(v); });

    return panel;
}

void PlotterPage::setSidePanelVisible(bool visible)
{
    m_sideScroll->setVisible(visible);
}

void PlotterPage::onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                              const QString &raw, const QHash<QString, double> &fields)
{
    m_plot->addSample(tSec, vals);
    m_logRows.push_back({raw, fields});
}

void PlotterPage::onMessage(const QString &raw)
{
    m_logView->appendPlainText(raw);
    m_logRows.push_back({raw, {}});
}

void PlotterPage::sendCommand(const QString &command)
{
    const QString text = command.trimmed();
    if (text.isEmpty())
        return;
    auto &bridge = SerialBridge::instance();
    if (!bridge.isConnected()) {
        m_statusLabel->setText(QStringLiteral("Connect to the USB serial port first"));
        return;
    }
    bridge.writeLine(text);
    m_statusLabel->setText(QStringLiteral("Sent: %1").arg(text));
}

void PlotterPage::onSendClicked()
{
    sendCommand(m_commandEdit->text());
    m_commandEdit->clear();
}

void PlotterPage::onClearClicked()
{
    m_plot->clear();
    m_logRows.clear();
    m_logView->clear();
    m_statusLabel->setText(QStringLiteral("Cleared graphs and serial monitor"));
}

void PlotterPage::onSaveCsv()
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
    m_statusLabel->setText(QStringLiteral("Saved %1 log lines").arg(m_logRows.size()));
}
