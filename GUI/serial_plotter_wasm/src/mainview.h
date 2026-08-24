// The combined Live-Plotter + PID-Tuner view. Both share ONE LivePlot and ONE
// log; switching between them only swaps the left control panel (via a
// QStackedWidget), so the graphs keep running underneath.
#pragma once

#include <QHash>
#include <QString>
#include <QTimer>
#include <array>
#include <deque>

#include <QWidget>

#include "channels.h"
#include "logevent.h"

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QScrollArea;
class QStackedWidget;
QT_END_NAMESPACE

class LivePlot;

class MainView : public QWidget
{
    Q_OBJECT
public:
    enum Panel { PlotterPanel = 0, TunerPanel = 1 };

    explicit MainView(double windowS, QWidget *parent = nullptr);

    void showPanel(Panel panel);
    void setSidePanelVisible(bool visible);

private slots:
    void onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                     const QString &raw, const QHash<QString, double> &fields);
    void onLogEvent(const logevt::Event &event);
    void onConnectionChanged(bool connected, const QString &message);

    // Plotter panel
    void onSendClicked();
    void onSaveCsv();
    void onRecordToggled(bool on);

    // Tuner panel
    void onApplySetpoint();
    void onStopSetpoint();
    void onReadConfig();
    void onConfigReceived(const QHash<QString, double> &fields);

    void clearAll();

private:
    QWidget *buildPlotterPanel(double windowS);
    QWidget *buildTunerPanel();
    void sendCommand(const QString &command);
    void setStatus(const QString &text);
    QString setpointPrefix() const;

    // Raw lines only, re-parsed when exporting. Keeping the parsed QHash here
    // meant allocating ~10 QString keys per telemetry line, forever -- the main
    // source of the WASM build's memory churn. Bounded so a long session can't
    // grow without limit; the oldest lines fall off the front.
    void appendLogRow(const QString &raw);
    void updateRecordIndicator();
    std::deque<QString> m_logRows;
    bool m_wasConnected = false;
    // CSV capture is opt-in: nothing is retained until Start is pressed. Also
    // means an idle session costs nothing to keep open.
    bool m_recording = false;
    // The row count is refreshed on a timer rather than per line: at 10 Hz,
    // reformatting the label on every sample is exactly the kind of churn the
    // rest of this class avoids.
    QTimer m_recordTimer;

    QWidget *m_leftColumn = nullptr;
    QStackedWidget *m_leftStack = nullptr;
    LivePlot *m_plot = nullptr;
    QPlainTextEdit *m_logView = nullptr;
    QLabel *m_statusLabel = nullptr;

    // Plotter widgets
    QDoubleSpinBox *m_windowSpin = nullptr;
    QLineEdit *m_commandEdit = nullptr;
    QPushButton *m_recordButton = nullptr;
    QPushButton *m_saveButton = nullptr;
    QLabel *m_recordIndicator = nullptr;
    // Which severities reach the monitor pane. A VIEW filter: the lines are
    // still received and still recorded to CSV, they are just not drawn. The
    // source-side filter (how much the board sends at all) is the station's
    // D<n> command, on the CAN Devices page.
    QComboBox *m_logFilterCombo = nullptr;

    // Tuner widgets
    QComboBox *m_modeCombo = nullptr;
    QDoubleSpinBox *m_setpointSpin = nullptr;

    // The three PID loops are identical in everything but which cfg keys they
    // read and write, so they are one array rather than nine members and three
    // near-identical slots. The keys index the shared config schema, which is
    // where the write command ("KP") and the decimals come from -- so the tuner
    // and the Motor Config page always send a gain the same way.
    struct GainGroup {
        const char *title;
        const char *keys[3];                 // cfg keys, in KP / KI / KD order
        QDoubleSpinBox *spin[3] = {nullptr, nullptr, nullptr};
    };
    GainGroup m_gains[3] = {
        {"Velocity PID gains", {"vel_p", "vel_i", "vel_d"}, {}},
        {"Current PID gains",  {"cur_p", "cur_i", "cur_d"}, {}},
        {"Position PID gains", {"pos_gain", "pos_i", "pos_d"}, {}},
    };
    void applyGains(int group);              // push one loop's trio to the board

    QLabel *m_configSummary = nullptr;       // live config read-back (Q)
};
