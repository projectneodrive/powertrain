#include "plotwindow.h"
#include "demosource.h"
#include "serialbridge.h"
#include "telemetry.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QAction>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPlainTextEdit>
#include <QKeySequence>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSpinBox>
#include <QSplitter>
#include <QTextStream>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

// QChartView derives from QGraphicsView, which accepts wheel events even when
// it has nothing of its own to scroll. That silently prevents the enclosing
// QScrollArea from ever seeing them, so the plot column looks "unscrollable".
//
// Merely calling event->ignore() is not enough: automatic propagation to the
// parent is unreliable here (and more so under WebAssembly), so we drive the
// enclosing scroll area's scrollbar directly.
class ChartView : public QChartView
{
public:
    explicit ChartView(QChart *chart) : QChartView(chart)
    {
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }

protected:
    void wheelEvent(QWheelEvent *event) override
    {
        for (QWidget *w = parentWidget(); w; w = w->parentWidget()) {
            if (auto *area = qobject_cast<QScrollArea *>(w)) {
                QScrollBar *bar = area->verticalScrollBar();
                bar->setValue(bar->value() - event->angleDelta().y());
                event->accept();
                return;
            }
        }
        event->ignore();
    }
};

struct ChannelDef {
    const char *label;
    const char *color;
    const char *primaryKey;
    const char *altKey;      // fallback key name, or nullptr
};

// Matches PLOT_DEFS in the Python plotter and the field names emitted by
// SerialTask in src/main.cpp.
const ChannelDef kChannels[kNumChannels] = {
    {"Target",      "#f97316", "tgt",  nullptr},
    {"Iq [A]",      "#22c55e", "Iq",   "iq"},
    {"Vel [rad/s]", "#3b82f6", "vel",  nullptr},
    {"Pos [rad]",   "#a855f7", "pos",  nullptr},
    {"Vbus [V]",    "#ef4444", "Vbus", "vbus"},
};

constexpr double kMaxHistoryS = 300.0;   // hard cap on retained samples
constexpr int kMinPlotHeight = 220;      // per-chart floor before scrolling

} // namespace

PlotWindow::PlotWindow(double windowS, QWidget *parent)
    : QMainWindow(parent), m_windowS(windowS)
{
    setWindowTitle(QStringLiteral("Powertrain live plot (WASM)"));
    resize(1280, 860);

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *mainLayout = new QHBoxLayout(central);

    auto *splitter = new QSplitter(Qt::Horizontal);
    mainLayout->addWidget(splitter);

    // Both halves scroll independently: the controls column is taller than a
    // short window, and the five stacked charts need room to stay readable
    // rather than being squashed to nothing.
    m_leftScroll = new QScrollArea;
    m_leftScroll->setWidgetResizable(true);
    m_leftScroll->setFrameShape(QFrame::NoFrame);
    m_leftScroll->setWidget(buildLeftPanel());

    m_plotScroll = new QScrollArea;
    m_plotScroll->setWidgetResizable(true);
    m_plotScroll->setFrameShape(QFrame::NoFrame);
    m_plotScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_plotScroll->setWidget(buildPlots());

    splitter->addWidget(m_leftScroll);
    splitter->addWidget(m_plotScroll);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({380, 900});

    // The toggle must live outside the panel, or hiding it would take the
    // button away with it.
    auto *toolbar = addToolBar(QStringLiteral("View"));
    toolbar->setMovable(false);
    m_togglePanelAction = toolbar->addAction(QStringLiteral("Hide panel"));
    m_togglePanelAction->setCheckable(true);
    m_togglePanelAction->setChecked(true);
    m_togglePanelAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+B")));
    m_togglePanelAction->setToolTip(QStringLiteral("Show/hide the controls panel (Ctrl+B)"));
    connect(m_togglePanelAction, &QAction::toggled, this, &PlotWindow::onTogglePanel);

    m_demo = new DemoSource(this);

    auto &bridge = SerialBridge::instance();
    connect(&bridge, &SerialBridge::lineReceived, this, &PlotWindow::onLineReceived);
    connect(&bridge, &SerialBridge::statusChanged, this, &PlotWindow::onStatusChanged);
}

void PlotWindow::enableDemo(bool on)
{
    m_demoCheck->setChecked(on);
}

void PlotWindow::dumpScrollDiagnostics() const
{
    const QWidget *content = m_plotScroll->widget();
    QString out;
    QTextStream s(&out);
    s << "--- plot scroll ---\n"
      << " content minimumSizeHint : " << content->minimumSizeHint().height() << "\n"
      << " content actual height   : " << content->height() << "\n"
      << " viewport height         : " << m_plotScroll->viewport()->height() << "\n"
      << " vbar max                : " << m_plotScroll->verticalScrollBar()->maximum() << "\n"
      << " SCROLLS                 : "
      << (m_plotScroll->verticalScrollBar()->maximum() > 0 ? "YES" : "NO") << "\n"
      << "--- left scroll ---\n"
      << " content minimumSizeHint : " << m_leftScroll->widget()->minimumSizeHint().height() << "\n"
      << " viewport height         : " << m_leftScroll->viewport()->height() << "\n"
      << " vbar max                : " << m_leftScroll->verticalScrollBar()->maximum() << "\n"
      << " SCROLLS                 : "
      << (m_leftScroll->verticalScrollBar()->maximum() > 0 ? "YES" : "NO") << "\n";

    QFile f(QStringLiteral("selftest.txt"));
    if (f.open(QIODevice::WriteOnly | QIODevice::Text))
        f.write(out.toUtf8());
}

// ------------------------------------------------------------------- UI --

QWidget *PlotWindow::buildLeftPanel()
{
    auto *panel = new QWidget;
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    // Connection ---------------------------------------------------------
    auto *connGroup = new QGroupBox(QStringLiteral("Connection"));
    auto *connLayout = new QGridLayout(connGroup);
    m_baudSpin = new QSpinBox;
    m_baudSpin->setRange(300, 4000000);
    m_baudSpin->setValue(115200);
    m_connectButton = new QPushButton(QStringLiteral("Connect (USB)"));
    connLayout->addWidget(new QLabel(QStringLiteral("Baud")), 0, 0);
    connLayout->addWidget(m_baudSpin, 0, 1);
    connLayout->addWidget(m_connectButton, 1, 0, 1, 2);
    auto *hint = new QLabel(QStringLiteral(
        "Click Connect, then pick the ODrive's USB serial port in the browser prompt."));
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral("color:#666; font-size:11px;"));
    connLayout->addWidget(hint, 2, 0, 1, 2);
    m_demoCheck = new QCheckBox(QStringLiteral("Demo data (no hardware)"));
    m_demoCheck->setToolTip(QStringLiteral(
        "Feed synthetic firmware-style telemetry through the normal parse path."));
    connLayout->addWidget(m_demoCheck, 3, 0, 1, 2);

    // Controls -----------------------------------------------------------
    auto *ctrlGroup = new QGroupBox(QStringLiteral("Controls"));
    auto *ctrlLayout = new QGridLayout(ctrlGroup);
    m_windowSpin = new QDoubleSpinBox;
    m_windowSpin->setRange(1.0, kMaxHistoryS);
    m_windowSpin->setSingleStep(1.0);
    m_windowSpin->setDecimals(1);
    m_windowSpin->setValue(m_windowS);
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
    // The build stamp makes a stale, browser-cached .wasm obvious at a glance.
    m_statusLabel = new QLabel(QStringLiteral("Disconnected — build %1 %2")
                                   .arg(QString::fromLatin1(__DATE__),
                                        QString::fromLatin1(__TIME__)));
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
    // Without a floor the log box collapses and the panel never needs to
    // scroll; with one the panel scrolls instead of squashing the monitor.
    m_logView->setMinimumHeight(200);
    logsLayout->addWidget(m_logView);

    layout->addWidget(connGroup);
    layout->addWidget(ctrlGroup);
    layout->addWidget(cmdGroup);
    layout->addWidget(statusGroup);
    layout->addWidget(logsGroup);
    layout->addStretch(1);
    panel->setMinimumWidth(340);

    // signals
    connect(m_connectButton, &QPushButton::clicked, this, &PlotWindow::onConnectClicked);
    connect(m_demoCheck, &QCheckBox::toggled, this, &PlotWindow::onDemoToggled);
    connect(clearButton, &QPushButton::clicked, this, &PlotWindow::onClearClicked);
    connect(saveButton, &QPushButton::clicked, this, &PlotWindow::onSaveCsv);
    connect(sendButton, &QPushButton::clicked, this, &PlotWindow::onSendClicked);
    connect(m_commandEdit, &QLineEdit::returnPressed, this, &PlotWindow::onSendClicked);
    connect(m_windowSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PlotWindow::onWindowChanged);

    return panel;
}

QWidget *PlotWindow::buildPlots()
{
    auto *container = new QWidget;
    auto *layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);
    // Belt and braces: state the container's own floor rather than relying
    // solely on the per-chart minimums propagating through the layout.
    container->setMinimumHeight(kMinPlotHeight * kNumChannels);

    for (int ch = 0; ch < kNumChannels; ++ch) {
        auto *series = new QLineSeries;
        series->setColor(QColor(QString::fromLatin1(kChannels[ch].color)));
        series->setName(QString::fromLatin1(kChannels[ch].label));

        auto *chart = new QChart;
        chart->legend()->hide();
        chart->addSeries(series);
        chart->setMargins(QMargins(4, 2, 8, 2));

        auto *axX = new QValueAxis;
        axX->setRange(0.0, m_windowS);
        axX->setLabelFormat(QStringLiteral("%.0f"));
        auto *axY = new QValueAxis;
        axY->setTitleText(QString::fromLatin1(kChannels[ch].label));
        axY->setRange(-1.0, 1.0);

        chart->addAxis(axX, Qt::AlignBottom);
        chart->addAxis(axY, Qt::AlignLeft);
        series->attachAxis(axX);
        series->attachAxis(axY);
        // Only the bottom chart shows the time axis labels.
        axX->setLabelsVisible(ch == kNumChannels - 1);
        if (ch == kNumChannels - 1)
            axX->setTitleText(QStringLiteral("Time [s]"));

        auto *view = new ChartView(chart);
        view->setRenderHint(QPainter::Antialiasing, false);
        // Floor per chart: once the viewport is shorter than 5x this, the
        // enclosing QScrollArea scrolls rather than shrinking them further.
        // Kept generous so each trace stays readable on a laptop screen --
        // with 5 plots this is deliberately taller than most browser viewports.
        view->setMinimumHeight(kMinPlotHeight);
        layout->addWidget(view);

        m_series[ch] = series;
        m_charts[ch] = chart;
        m_axX[ch] = axX;
        m_axY[ch] = axY;
    }

    return container;
}

// -------------------------------------------------------------- actions --

void PlotWindow::onConnectClicked()
{
    auto &bridge = SerialBridge::instance();
    if (bridge.isConnected())
        bridge.disconnectPort();
    else
        bridge.connectPort(m_baudSpin->value());
}

void PlotWindow::onTogglePanel(bool visible)
{
    m_leftScroll->setVisible(visible);
    m_togglePanelAction->setText(visible ? QStringLiteral("Hide panel")
                                         : QStringLiteral("Show panel"));
}

void PlotWindow::onDemoToggled(bool on)
{
    if (on) {
        onClearClicked();
        m_demo->start();
        setStatusText(QStringLiteral("Demo mode â€” synthetic telemetry, no hardware"));
    } else {
        m_demo->stop();
        setStatusText(QStringLiteral("Demo stopped"));
    }
    // The real port and the generator would interleave into one stream.
    m_connectButton->setEnabled(!on);
}

void PlotWindow::onStatusChanged(bool connected, const QString &message)
{
    m_connectButton->setText(connected ? QStringLiteral("Disconnect")
                                       : QStringLiteral("Connect (USB)"));
    m_baudSpin->setEnabled(!connected);
    m_demoCheck->setEnabled(!connected);
    setStatusText(message);
}

void PlotWindow::onLineReceived(const QString &line)
{
    const QHash<QString, double> fields = telemetry::parseLine(line);

    // Firmware text/log lines (no 't=') go to the monitor only.
    if (!fields.contains(QStringLiteral("t"))) {
        appendLog(line);
        m_logRows.push_back({line, fields});
        return;
    }
    m_logRows.push_back({line, fields});

    const double tMs = fields.value(QStringLiteral("t"));
    if (!m_haveT0) {
        m_t0 = tMs / 1000.0;
        m_haveT0 = true;
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
    addSample(tMs / 1000.0 - m_t0, vals);
}

void PlotWindow::addSample(double t, const std::array<double, kNumChannels> &vals)
{
    m_samples.push_back({t, vals});

    // Drop anything older than the hard history cap.
    const double cutoff = t - kMaxHistoryS;
    while (!m_samples.empty() && m_samples.front().t < cutoff)
        m_samples.pop_front();

    redraw();
}

void PlotWindow::redraw()
{
    if (m_samples.empty())
        return;

    const double tLast = m_samples.back().t;
    const double tStart = tLast - m_windowS;

    // Find the first sample inside the visible window (samples are ordered).
    auto it = std::lower_bound(
        m_samples.begin(), m_samples.end(), tStart,
        [](const Sample &s, double v) { return s.t < v; });

    std::array<QList<QPointF>, kNumChannels> points;
    std::array<double, kNumChannels> yMin;
    std::array<double, kNumChannels> yMax;
    yMin.fill(std::numeric_limits<double>::max());
    yMax.fill(std::numeric_limits<double>::lowest());

    for (auto s = it; s != m_samples.end(); ++s) {
        for (int ch = 0; ch < kNumChannels; ++ch) {
            const double y = s->v[ch];
            points[ch].append(QPointF(s->t, y));
            yMin[ch] = std::min(yMin[ch], y);
            yMax[ch] = std::max(yMax[ch], y);
        }
    }

    const double xLo = std::max(0.0, tStart);
    const double xHi = std::max(m_windowS, tLast);
    for (int ch = 0; ch < kNumChannels; ++ch) {
        m_series[ch]->replace(points[ch]);
        m_axX[ch]->setRange(xLo, xHi);
        double lo = yMin[ch];
        double hi = yMax[ch];
        if (lo > hi) { lo = -1.0; hi = 1.0; }         // no points
        double pad = (hi - lo) * 0.1;
        if (pad < 1e-6)
            pad = std::max(0.5, std::abs(hi) * 0.1);
        m_axY[ch]->setRange(lo - pad, hi + pad);
    }
}

void PlotWindow::sendCommand(const QString &command)
{
    const QString text = command.trimmed();
    if (text.isEmpty())
        return;
    auto &bridge = SerialBridge::instance();
    if (!bridge.isConnected()) {
        setStatusText(QStringLiteral("Connect to the USB serial port before sending commands"));
        return;
    }
    bridge.writeLine(text);
    setStatusText(QStringLiteral("Sent: %1").arg(text));
}

void PlotWindow::onSendClicked()
{
    sendCommand(m_commandEdit->text());
    m_commandEdit->clear();
}

void PlotWindow::onClearClicked()
{
    m_samples.clear();
    m_logRows.clear();
    m_haveT0 = false;
    m_logView->clear();
    for (int ch = 0; ch < kNumChannels; ++ch) {
        m_series[ch]->clear();
        m_axX[ch]->setRange(0.0, m_windowS);
        m_axY[ch]->setRange(-1.0, 1.0);
    }
    setStatusText(QStringLiteral("Cleared graphs and serial monitor"));
}

void PlotWindow::onSaveCsv()
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
    // In the browser this triggers a download of the generated file.
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
    setStatusText(QStringLiteral("Saved %1 log lines").arg(m_logRows.size()));
}

void PlotWindow::onWindowChanged(double value)
{
    m_windowS = value;
    redraw();
}

void PlotWindow::appendLog(const QString &line)
{
    m_logView->appendPlainText(line);
}

void PlotWindow::setStatusText(const QString &text)
{
    m_statusLabel->setText(text);
}

