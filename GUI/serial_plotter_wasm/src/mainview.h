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
    void onMessage(const QString &raw);
    void onConnectionChanged(bool connected, const QString &message);

    // Plotter panel
    void onSendClicked();
    void onSaveCsv();
    void onRecordToggled(bool on);

    // Tuner panel
    void onApplySetpoint();
    void onStopSetpoint();
    void onApplyGains();
    void onApplyCurrentGains();
    void onApplyPositionGains();
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

    // Tuner widgets
    QComboBox *m_modeCombo = nullptr;
    QDoubleSpinBox *m_setpointSpin = nullptr;
    QDoubleSpinBox *m_kpSpin = nullptr;      // velocity PID
    QDoubleSpinBox *m_kiSpin = nullptr;
    QDoubleSpinBox *m_kdSpin = nullptr;
    QDoubleSpinBox *m_curKpSpin = nullptr;   // current PID
    QDoubleSpinBox *m_curKiSpin = nullptr;
    QDoubleSpinBox *m_curKdSpin = nullptr;
    QDoubleSpinBox *m_posKpSpin = nullptr;   // position PID
    QDoubleSpinBox *m_posKiSpin = nullptr;
    QDoubleSpinBox *m_posKdSpin = nullptr;
    QLabel *m_configSummary = nullptr;       // live config read-back (Q)
};
