#pragma once

#include <QMainWindow>
#include <QHash>
#include <QString>
#include <QVector>
#include <array>
#include <deque>

QT_BEGIN_NAMESPACE
class QChart;
class QChartView;
class QLineSeries;
class QValueAxis;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;
class QLineEdit;
class QLabel;
class QPlainTextEdit;
QT_END_NAMESPACE

// Number of plotted channels: Target, Iq, Vel, Pos, Vbus.
static constexpr int kNumChannels = 5;

class PlotWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit PlotWindow(double windowS, QWidget *parent = nullptr);

private slots:
    void onConnectClicked();
    void onStatusChanged(bool connected, const QString &message);
    void onLineReceived(const QString &line);
    void onSendClicked();
    void onClearClicked();
    void onSaveCsv();
    void onWindowChanged(double value);

private:
    struct Sample {
        double t;                       // seconds, relative to first sample
        std::array<double, kNumChannels> v;
    };
    struct LogRow {
        QString raw;
        QHash<QString, double> fields;
    };

    QWidget *buildLeftPanel();
    QWidget *buildPlots();
    void sendCommand(const QString &command);
    void appendLog(const QString &line);
    void addSample(double t, const std::array<double, kNumChannels> &vals);
    void redraw();
    void setStatusText(const QString &text);

    // model / state
    double m_windowS;
    bool m_haveT0 = false;
    double m_t0 = 0.0;
    std::deque<Sample> m_samples;
    QVector<LogRow> m_logRows;

    // plotting
    std::array<QChart *, kNumChannels> m_charts{};
    std::array<QLineSeries *, kNumChannels> m_series{};
    std::array<QValueAxis *, kNumChannels> m_axX{};
    std::array<QValueAxis *, kNumChannels> m_axY{};

    // widgets
    QSpinBox *m_baudSpin = nullptr;
    QPushButton *m_connectButton = nullptr;
    QDoubleSpinBox *m_windowSpin = nullptr;
    QCheckBox *m_freezeCheck = nullptr;
    QLineEdit *m_commandEdit = nullptr;
    QLabel *m_statusLabel = nullptr;
    QPlainTextEdit *m_logView = nullptr;
};
