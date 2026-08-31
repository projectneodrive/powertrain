#include "candevicespage.h"

#include "axisvocab.h"
#include "serialbridge.h"
#include "telemetry.h"
#include "telemetryhub.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDateTime>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <array>

namespace {

// A "can ..." line arrives every second. Twice that plus a margin: past it the
// numbers on screen are history and saying so beats showing them as if live.
constexpr qint64 kStaleMs = 2500;

// Tags whose events belong on this page. AXIS is included because an axis fault
// is what the CAN link exists to report; the plotter's monitor still shows
// everything.
bool isCanTag(const QString &tag)
{
    return tag == QLatin1String("CAN") || tag == QLatin1String("BUS") ||
           tag == QLatin1String("LINK") || tag == QLatin1String("AXIS");
}

const char *kOk = "#16a34a";
const char *kBad = "#dc2626";
const char *kWarn = "#d97706";
const char *kMuted = "#6b7280";

} // namespace

CanDevicesPage::CanDevicesPage(QWidget *parent) : QWidget(parent)
{
    auto *outer = new QVBoxLayout(this);

    // ------------------------------------------------------------- header --
    m_header = new QLabel;
    QFont hf = m_header->font();
    hf.setPointSize(hf.pointSize() + 4);
    hf.setBold(true);
    m_header->setFont(hf);

    m_sourceLabel = new QLabel;
    m_sourceLabel->setWordWrap(true);
    m_sourceLabel->setStyleSheet(QStringLiteral("color:%1;font-size:11px;").arg(QLatin1String(kMuted)));

    outer->addWidget(m_header);
    outer->addWidget(m_sourceLabel);

    // -------------------------------------------------------------- tables --
    m_deviceTable = buildTable({QStringLiteral("Node id"),
                                QStringLiteral("Link"),
                                QStringLiteral("Axis state"),
                                QStringLiteral("Control mode"),
                                QStringLiteral("Heartbeat age"),
                                QStringLiteral("Heartbeat period"),
                                QStringLiteral("Worst gap (last 1 s)"),
                                QStringLiteral("Link timeout"),
                                QStringLiteral("Link drops"),
                                QStringLiteral("Worst scan (last 1 s)"),
                                QStringLiteral("Safety stop"),
                                QStringLiteral("Bit rate")});

    m_busTable = buildTable({QStringLiteral("Controller state"),
                             QStringLiteral("Frames sent"),
                             QStringLiteral("Send failures"),
                             QStringLiteral("Frames received"),
                             QStringLiteral("TX error counter"),
                             QStringLiteral("RX error counter"),
                             QStringLiteral("TX failed (driver)"),
                             QStringLiteral("RX missed"),
                             QStringLiteral("RX overrun"),
                             QStringLiteral("Arbitration lost"),
                             QStringLiteral("Bus error count")});

    m_errorTable = buildTable({QStringLiteral("Axis"),
                               QStringLiteral("Motor"),
                               QStringLiteral("Encoder"),
                               QStringLiteral("Controller")});

    // The tables are fixed-height, so without the trailing stretch the layout
    // centres them vertically -- and since the two side-by-side groups have
    // different row counts, the shorter one floats in the middle of its box
    // with a band of empty group above the header.
    auto group = [](const QString &title, QWidget *inner) {
        auto *g = new QGroupBox(title);
        auto *l = new QVBoxLayout(g);
        l->setContentsMargins(6, 6, 6, 6);
        l->addWidget(inner);
        l->addStretch(1);
        return g;
    };

    auto *topRow = new QHBoxLayout;
    topRow->addWidget(group(QStringLiteral("Device"), m_deviceTable), 1);
    topRow->addWidget(group(QStringLiteral("Bus — ESP32 TWAI controller"), m_busTable), 1);
    outer->addLayout(topRow);

    auto *errGroup = group(QStringLiteral("Error words"), m_errorTable);
    outer->addWidget(errGroup);

    m_nodesLabel = new QLabel;
    m_nodesLabel->setWordWrap(true);
    outer->addWidget(m_nodesLabel);

    // ----------------------------------------------------------- event log --
    auto *logGroup = new QGroupBox(QStringLiteral("CAN events"));
    auto *logLayout = new QVBoxLayout(logGroup);

    auto *filterRow = new QHBoxLayout;
    m_filterCombo = new QComboBox;
    m_filterCombo->addItem(QStringLiteral("Errors only"));
    m_filterCombo->addItem(QStringLiteral("Warnings and above"));
    m_filterCombo->addItem(QStringLiteral("Info and above"));
    m_filterCombo->addItem(QStringLiteral("Everything (incl. frame trace)"));
    m_filterCombo->setCurrentIndex(logevt::Info);
    filterRow->addWidget(new QLabel(QStringLiteral("Show")));
    filterRow->addWidget(m_filterCombo, 1);
    auto *clearLog = new QPushButton(QStringLiteral("Clear"));
    filterRow->addWidget(clearLog);
    logLayout->addLayout(filterRow);

    m_logView = new QPlainTextEdit;
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(500);
    m_logView->setMinimumHeight(120);
    logLayout->addWidget(m_logView);
    outer->addWidget(logGroup, 1);

    m_statusLabel = new QLabel;
    m_statusLabel->setStyleSheet(QStringLiteral("color:%1;").arg(QLatin1String(kMuted)));
    outer->addWidget(m_statusLabel);

    connect(clearLog, &QPushButton::clicked, m_logView, &QPlainTextEdit::clear);

    // ------------------------------------------------------------- wiring --
    auto &hub = TelemetryHub::instance();
    connect(&hub, &TelemetryHub::canStatus, this, &CanDevicesPage::onCanStatus);
    connect(&hub, &TelemetryHub::telemetry, this, &CanDevicesPage::onTelemetry);
    connect(&hub, &TelemetryHub::logEvent, this, &CanDevicesPage::onLogEvent);
    connect(&SerialBridge::instance(), &SerialBridge::statusChanged,
            this, &CanDevicesPage::onConnectionChanged);

    m_staleTimer.setInterval(1000);
    connect(&m_staleTimer, &QTimer::timeout, this, &CanDevicesPage::checkStale);
    m_staleTimer.start();

    showNoLink();
}

// ---------------------------------------------------------------------------
QTableWidget *CanDevicesPage::buildTable(const QStringList &rowLabels)
{
    auto *t = new QTableWidget(rowLabels.size(), 1, this);
    t->setVerticalHeaderLabels(rowLabels);
    t->setHorizontalHeaderLabels({QStringLiteral("Value")});
    t->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    t->setSelectionMode(QAbstractItemView::NoSelection);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    t->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    for (int r = 0; r < rowLabels.size(); ++r)
        t->setItem(r, 0, new QTableWidgetItem(QStringLiteral("--")));
    t->resizeRowsToContents();

    // Pinned to exactly its content height, measured rather than assumed: a
    // hard-coded row height is wrong on a HiDPI display, and getting it wrong
    // either leaves a band of empty grid under the last row (which pushed the
    // buttons off the bottom of the page) or hides a row behind a scrollbar
    // inside a page that is already scrolling.
    t->setFixedHeight(t->horizontalHeader()->height() + t->verticalHeader()->length() +
                      2 * t->frameWidth());
    t->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return t;
}

void CanDevicesPage::setCell(QTableWidget *table, int row, const QString &text,
                             const QString &color)
{
    QTableWidgetItem *item = table->item(row, 0);
    if (!item)
        return;
    item->setText(text);
    if (color.isEmpty())
        item->setData(Qt::ForegroundRole, QVariant());   // back to the palette
    else
        item->setForeground(QColor(color));
}

void CanDevicesPage::showNoLink()
{
    m_header->setText(QStringLiteral("No CAN status"));
    m_header->setStyleSheet(QStringLiteral("color:%1;").arg(QLatin1String(kMuted)));
    m_sourceLabel->setText(QStringLiteral(
        "Waiting for a \"can …\" status line. That line comes from the ESP32 control "
        "station (can_utilities) — connect to ITS USB port to populate this page. "
        "Connected straight to the board instead, only the frame counters below are "
        "available, from its telemetry line."));
    m_nodesLabel->setText(QString());
}

// ---------------------------------------------------------------------------
void CanDevicesPage::onCanStatus(const QHash<QString, QString> &f)
{
    using telemetry::tokenUInt;

    m_haveCanLine = true;
    m_lastCanMs = QDateTime::currentMSecsSinceEpoch();

    const quint64 node = tokenUInt(f, "node");
    const bool linkUp = tokenUInt(f, "link") != 0;
    const quint64 hbAge = tokenUInt(f, "hb_age");
    const quint64 baud = tokenUInt(f, "baud");
    const quint32 axisErr = quint32(tokenUInt(f, "axis_err"));

    m_header->setText(linkUp
        ? QStringLiteral("● Node %1 — LINK UP").arg(node)
        : QStringLiteral("○ Node %1 — LINK DOWN").arg(node));
    m_header->setStyleSheet(QStringLiteral("color:%1;")
                                .arg(QLatin1String(linkUp ? (axisErr ? kWarn : kOk) : kBad)));
    m_sourceLabel->setText(QStringLiteral(
        "Live from the ESP32 control station, once a second."));

    setCell(m_deviceTable, DevNode, QString::number(node));
    setCell(m_deviceTable, DevLink, linkUp ? QStringLiteral("UP") : QStringLiteral("DOWN"),
            QLatin1String(linkUp ? kOk : kBad));
    setCell(m_deviceTable, DevState, axisvocab::state(unsigned(tokenUInt(f, "axis"))));
    setCell(m_deviceTable, DevMode, axisvocab::mode(unsigned(tokenUInt(f, "mode"))));
    // Age is the field that makes "DOWN" actionable: 600 ms means it just
    // dropped, 90 s means it was never there.
    setCell(m_deviceTable, DevHbAge, QStringLiteral("%1 ms").arg(hbAge),
            QLatin1String(linkUp ? "" : kBad));
    const quint64 hbPeriod = tokenUInt(f, "hb_period");
    setCell(m_deviceTable, DevHbPeriod, QStringLiteral("%1 ms").arg(hbPeriod));

    // The row that says WHY the link keeps dropping. While the link is up a gap
    // can never reach the timeout — exceeding it is what declares the loss — so
    // this measures how ragged the link is *while it works*:
    //   near the heartbeat period  -> clean; a drop means the sender stopped
    //   well above it              -> frames already being lost; suspect wiring
    const quint64 hbMax = tokenUInt(f, "hb_max");
    const quint64 drops = tokenUInt(f, "drops");
    QString worst = hbMax ? QStringLiteral("%1 ms").arg(hbMax) : QStringLiteral("--");
    QString worstColor;
    if (hbPeriod && hbMax > hbPeriod * 2) {
        worst += QStringLiteral("  — frames being lost, suspect wiring/termination");
        worstColor = QLatin1String(kWarn);
    } else if (hbPeriod && hbMax && drops) {
        worst += QStringLiteral("  — clean while up, so drops are the sender stopping");
    }
    setCell(m_deviceTable, DevHbWorst, worst, worstColor);

    setCell(m_deviceTable, DevTimeout, QStringLiteral("%1 ms").arg(tokenUInt(f, "hb_timeout")));

    setCell(m_deviceTable, DevDrops, QStringLiteral("%1 since boot").arg(drops),
            drops ? QLatin1String(kWarn) : QString());

    // Is the station's own loop keeping up? A healthy scan is under a
    // millisecond. Approaching the heartbeat period means the link checks are
    // sampling too coarsely to be trusted -- which is what the frame trace at
    // D3 does, and why it used to make the axis drop out.
    const quint64 scanMax = telemetry::tokenUInt(f, "scan_max");
    QString scan = QStringLiteral("%1 ms").arg(scanMax);
    QString scanColor;
    if (hbPeriod && scanMax >= hbPeriod) {
        scan += QStringLiteral("  — loop is not keeping up; lower the log level");
        scanColor = QLatin1String(kWarn);
    }
    setCell(m_deviceTable, DevScanMax, scan, scanColor);

    // The one-shot link-loss stop. Worth its own row because a disarmed axis
    // and an axis nobody armed look identical from the outside — this is the
    // row that says which one you are looking at.
    const quint64 stopAfter = tokenUInt(f, "stop_after");
    const bool stopped = tokenUInt(f, "stopped") != 0;
    if (stopAfter == 0) {
        setCell(m_deviceTable, DevSafetyStop, QStringLiteral("disabled"), QLatin1String(kMuted));
    } else if (stopped) {
        setCell(m_deviceTable, DevSafetyStop,
                QStringLiteral("FIRED — axis disarmed, press Arm (A) once the link is back"),
                QLatin1String(kBad));
    } else {
        setCell(m_deviceTable, DevSafetyStop,
                QStringLiteral("armed — disarms %1 s after the heartbeat stops")
                    .arg(stopAfter / 1000.0, 0, 'f', 1));
    }

    setCell(m_deviceTable, DevBaud, QStringLiteral("%1 kbit/s").arg(baud / 1000));

    // Bus. A non-zero error counter while the controller still reports RUNNING
    // is the missing-terminator signature, so those rows are tinted rather than
    // left to be spotted in a column of zeros.
    const quint64 busState = tokenUInt(f, "bus");
    static const char *kBusNames[] = {"STOPPED", "RUNNING", "BUS_OFF", "RECOVERING"};
    setCell(m_busTable, BusState,
            busState < 4 ? QLatin1String(kBusNames[busState]) : QStringLiteral("?"),
            QLatin1String(busState == 1 ? kOk : kBad));

    auto counter = [&](BusRow row, const char *key, bool badWhenNonZero) {
        const quint64 v = tokenUInt(f, key);
        setCell(m_busTable, row, QString::number(v),
                (badWhenNonZero && v) ? QLatin1String(kWarn) : QString());
    };
    counter(BusTxOk,      "tx_ok",      false);
    counter(BusTxFail,    "tx_fail",    true);
    counter(BusRx,        "rx",         false);
    counter(BusTxEc,      "tx_ec",      true);
    counter(BusRxEc,      "rx_ec",      true);
    counter(BusFailed,    "tx_failed",  true);
    counter(BusMissed,    "rx_missed",  true);
    counter(BusOverrun,   "rx_overrun", true);
    counter(BusArbLost,   "arb_lost",   false);   // normal on a shared bus
    counter(BusErrCount,  "bus_ec",     true);

    // Errors, decoded from the firmware-shared axis_vocab.h table.
    auto errorRow = [&](ErrRow row, const char *key, bool decode) {
        const quint32 v = quint32(tokenUInt(f, key));
        QString text = QStringLiteral("0x%1").arg(v, 0, 16);
        if (!decode) {
            text += QStringLiteral("  — reserved, this firmware reports through Axis");
        } else if (v == 0) {
            text += QStringLiteral("  (none)");
        } else {
            text += QStringLiteral("  %1").arg(axisvocab::errors(v));
        }
        setCell(m_errorTable, row, text,
                (decode && v) ? QLatin1String(kBad)
                              : (decode ? QString() : QLatin1String(kMuted)));
    };
    errorRow(ErrAxis, "axis_err", true);
    // These three are RESERVED and always zero. The firmware reports every
    // condition through the axis word, for which axis_vocab.h defines shared
    // names; the sub-words exist only so the ODrive Get_*_Error frames are
    // well-formed. Showing a bare "0x0" invites the reader to conclude the
    // subsystem is healthy, which is not what it means — say so instead.
    errorRow(ErrMotor, "motor_err", false);
    errorRow(ErrEncoder, "enc_err", false);
    errorRow(ErrController, "ctrl_err", false);

    // Node discovery. Anything other than the target is a node id mismatch or a
    // second device sharing the bus, and both are worth seeing without turning
    // the frame trace on.
    const quint64 mask = tokenUInt(f, "nodes");
    QStringList seen;
    for (int i = 0; i < 64; ++i) {
        if (!(mask & (quint64(1) << i)))
            continue;
        seen << (quint64(i) == node ? QStringLiteral("<b>%1 (target)</b>").arg(i)
                                    : QStringLiteral("%1").arg(i));
    }
    m_nodesLabel->setText(seen.isEmpty()
        ? QStringLiteral("Nodes seen on the bus: <i>none yet</i>")
        : QStringLiteral("Nodes seen on the bus: %1").arg(seen.join(QStringLiteral(", "))));
}

void CanDevicesPage::onTelemetry(double, const std::array<double, kNumChannels> &,
                                 const QString &, const QHash<QString, double> &fields)
{
    // Fallback for a direct USB connection to the board: it publishes its own
    // CAN counters on the telemetry line but nothing else about the link.
    if (m_haveCanLine)
        return;
    auto set = [&](BusRow row, const char *key) {
        const QString k = QString::fromLatin1(key);
        if (fields.contains(k))
            setCell(m_busTable, row, QString::number(qint64(fields.value(k))));
    };
    set(BusTxOk, "can_tx_ok");
    set(BusTxFail, "can_tx_fail");
    set(BusRx, "can_rx");
}

void CanDevicesPage::onLogEvent(const logevt::Event &e)
{
    if (!isCanTag(e.tag))
        return;
    if (int(e.level) > m_filterCombo->currentIndex())
        return;

    // Timestamped, because on this page the question is almost always "how
    // often" rather than "what" — a link dropping every 3.5 s is a board
    // rebooting, every 60 s is something thermal, and the text is identical.
    const QString when = logevt::timeLabel(e);
    m_logView->appendHtml(
        QStringLiteral("<span style='color:%1'>%2[%3] %4</span>")
            .arg(QLatin1String(logevt::levelColor(e.level)),
                 when.isEmpty() ? QString() : QStringLiteral("%1  ").arg(when, 10),
                 e.tag, e.text.toHtmlEscaped()));
}

void CanDevicesPage::onConnectionChanged(bool connected, const QString &message)
{
    m_statusLabel->setText(message);
    if (!connected) {
        // A disconnect invalidates every number on the page. Dropping the
        // have-a-line flag also re-enables the telemetry-line fallback, in case
        // the next connection is to the board rather than the station.
        m_haveCanLine = false;
        showNoLink();
    }
}

void CanDevicesPage::checkStale()
{
    if (!m_haveCanLine)
        return;
    if (QDateTime::currentMSecsSinceEpoch() - m_lastCanMs < kStaleMs)
        return;
    m_header->setText(m_header->text() + QStringLiteral("  (stale)"));
    m_header->setStyleSheet(QStringLiteral("color:%1;").arg(QLatin1String(kMuted)));
    m_sourceLabel->setText(QStringLiteral(
        "No \"can …\" line for over %1 s — the values below are the last ones received, "
        "not live.").arg(kStaleMs / 1000));
    m_haveCanLine = false;
}
