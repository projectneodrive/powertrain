// The combined Live-Plotter + PID-Tuner view. Both share ONE LivePlot and ONE
// log; switching between them only swaps the left control panel (via a
// QStackedWidget), so the graphs keep running underneath.
#pragma once

#include <QHash>
#include <QString>
#include <QVector>
#include <array>

#include <QWidget>

#include "channels.h"

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
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

    // Tuner panel
    void onApplySetpoint();
    void onStopSetpoint();
    void onApplyGains();

    void clearAll();

private:
    QWidget *buildPlotterPanel(double windowS);
    QWidget *buildTunerPanel();
    void sendCommand(const QString &command);
    void setStatus(const QString &text);
    QString setpointPrefix() const;

    struct LogRow {
        QString raw;
        QHash<QString, double> fields;
    };
    QVector<LogRow> m_logRows;
    bool m_wasConnected = false;

    QWidget *m_leftColumn = nullptr;
    QStackedWidget *m_leftStack = nullptr;
    LivePlot *m_plot = nullptr;
    QPlainTextEdit *m_logView = nullptr;
    QLabel *m_statusLabel = nullptr;

    // Plotter widgets
    QDoubleSpinBox *m_windowSpin = nullptr;
    QLineEdit *m_commandEdit = nullptr;

    // Tuner widgets
    QComboBox *m_modeCombo = nullptr;
    QDoubleSpinBox *m_setpointSpin = nullptr;
    QDoubleSpinBox *m_kpSpin = nullptr;
    QDoubleSpinBox *m_kiSpin = nullptr;
    QDoubleSpinBox *m_kdSpin = nullptr;
};
