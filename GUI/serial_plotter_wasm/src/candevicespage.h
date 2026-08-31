// CAN Devices — everything about the CAN link, on a page instead of in the log.
//
// It is fed by the "can ..." status line the ESP32 control station emits once a
// second (can_utilities/lib/can_bridge/bridge_telemetry.cpp). That line exists
// because this information is STATE, not events: node id, link, the four error
// words and nine bus counters are only meaningful as a live table, and the
// station used to print them as prose every two seconds into a pane showing
// about four seconds of history.
//
// It is a read-only monitor: the page reports link/bus/error state and a
// timestamped event log, and sends nothing itself (use the other pages, or the
// board's USB console, to actually command the axis).
//
// Plugged into the BOARD's USB port instead of the bridge, no "can ..." lines
// arrive. The page says so and falls back to the can_tx_ok/can_tx_fail/can_rx
// counters the firmware puts on its telemetry line, which is the subset it can
// honestly show.
#pragma once

#include <QHash>
#include <QString>
#include <QTimer>
#include <QWidget>
#include <array>

#include "channels.h"
#include "logevent.h"

QT_BEGIN_NAMESPACE
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QTableWidget;
QT_END_NAMESPACE

class CanDevicesPage : public QWidget
{
    Q_OBJECT
public:
    explicit CanDevicesPage(QWidget *parent = nullptr);

private slots:
    void onCanStatus(const QHash<QString, QString> &fields);
    void onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                     const QString &raw, const QHash<QString, double> &fields);
    void onLogEvent(const logevt::Event &event);
    void onConnectionChanged(bool connected, const QString &message);
    void checkStale();

private:
    QTableWidget *buildTable(const QStringList &rowLabels);
    void setCell(QTableWidget *table, int row, const QString &text,
                 const QString &color = QString());
    void showNoLink();

    // Device rows
    enum DevRow { DevNode, DevLink, DevState, DevMode, DevHbAge, DevHbPeriod,
                  DevHbWorst, DevTimeout, DevDrops, DevScanMax, DevSafetyStop, DevBaud,
                  DevRowCount };
    // Bus rows (the ESP32's own TWAI controller)
    enum BusRow { BusState, BusTxOk, BusTxFail, BusRx, BusTxEc, BusRxEc,
                  BusFailed, BusMissed, BusOverrun, BusArbLost, BusErrCount,
                  BusRowCount };
    // Error rows
    enum ErrRow { ErrAxis, ErrMotor, ErrEncoder, ErrController, ErrRowCount };

    QLabel *m_header = nullptr;
    QLabel *m_sourceLabel = nullptr;
    QLabel *m_nodesLabel = nullptr;
    QTableWidget *m_deviceTable = nullptr;
    QTableWidget *m_busTable = nullptr;
    QTableWidget *m_errorTable = nullptr;
    QPlainTextEdit *m_logView = nullptr;
    QComboBox *m_filterCombo = nullptr;
    QLabel *m_statusLabel = nullptr;

    // A "can ..." line has been seen this session -> the full table is live.
    bool m_haveCanLine = false;
    QTimer m_staleTimer;
    qint64 m_lastCanMs = 0;
};
