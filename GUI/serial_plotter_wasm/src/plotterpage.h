// The original live-plotter UI, now one page among several: a side panel
// (plot controls, serial command console, status, logs) next to a LivePlot.
// Connection is handled globally by the shell, not here.
#pragma once

#include <QHash>
#include <QString>
#include <QVector>

#include "apppage.h"
#include "channels.h"

QT_BEGIN_NAMESPACE
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QScrollArea;
QT_END_NAMESPACE

class LivePlot;

class PlotterPage : public AppPage
{
    Q_OBJECT
public:
    explicit PlotterPage(double windowS, QWidget *parent = nullptr);

    QString pageTitle() const override { return QStringLiteral("Live Plotter"); }
    bool hasSidePanel() const override { return true; }
    void setSidePanelVisible(bool visible) override;

private slots:
    void onTelemetry(double tSec, const std::array<double, kNumChannels> &vals,
                     const QString &raw, const QHash<QString, double> &fields);
    void onMessage(const QString &raw);
    void onSendClicked();
    void onClearClicked();
    void onSaveCsv();

private:
    QWidget *buildSidePanel(double windowS);
    void sendCommand(const QString &command);

    struct LogRow {
        QString raw;
        QHash<QString, double> fields;
    };
    QVector<LogRow> m_logRows;

    QScrollArea *m_sideScroll = nullptr;
    LivePlot *m_plot = nullptr;
    QDoubleSpinBox *m_windowSpin = nullptr;
    QLineEdit *m_commandEdit = nullptr;
    QLabel *m_statusLabel = nullptr;
    QPlainTextEdit *m_logView = nullptr;
};
