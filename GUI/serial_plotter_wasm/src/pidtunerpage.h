// Live control + feedback: arm/idle, pick a control mode, push setpoints and
// velocity-PID gains, and watch the response on a LivePlot with a log beneath.
// Every action maps to an existing serial command (A/I/C, V/T/X, KP/KI/KD).
#pragma once

#include <QHash>
#include <QString>
#include <array>

#include "apppage.h"
#include "channels.h"

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QScrollArea;
QT_END_NAMESPACE

class LivePlot;

class PidTunerPage : public AppPage
{
    Q_OBJECT
public:
    explicit PidTunerPage(double windowS, QWidget *parent = nullptr);

    QString pageTitle() const override { return QStringLiteral("PID Tuner"); }
    bool hasSidePanel() const override { return true; }
    void setSidePanelVisible(bool visible) override;

private slots:
    void onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                     const QString &raw, const QHash<QString, double> &fields);
    void onMessage(const QString &raw);
    void onApplySetpoint();
    void onStop();
    void onApplyGains();

private:
    QWidget *buildSidePanel();
    void send(const QString &command);
    QString setpointPrefix() const;   // "V", "T" or "X" for the current mode

    QScrollArea *m_sideScroll = nullptr;
    LivePlot *m_plot = nullptr;
    QPlainTextEdit *m_logView = nullptr;
    QLabel *m_statusLabel = nullptr;

    QComboBox *m_modeCombo = nullptr;
    QDoubleSpinBox *m_setpointSpin = nullptr;
    QDoubleSpinBox *m_kpSpin = nullptr;
    QDoubleSpinBox *m_kiSpin = nullptr;
    QDoubleSpinBox *m_kdSpin = nullptr;
};
