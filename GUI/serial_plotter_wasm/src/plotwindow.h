#pragma once

#include <QMainWindow>
#include <QHash>
#include <QString>
#include <QVector>
#include <array>
#include <deque>

QT_BEGIN_NAMESPACE
class QAction;
class QChart;
class QChartView;
class QLineSeries;
class QScrollArea;
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

class DemoSource;

class PlotWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit PlotWindow(double windowS, QWidget *parent = nullptr);

    // Start with synthetic telemetry (the --demo command line switch).
    void enableDemo(bool on);

    // Prints scroll-area geometry; used by the PLOTTER_SELFTEST harness.
    void dumpScrollDiagnostics() const;

private slots:
    void onConnectClicked();
    void onDemoToggled(bool on);
    void onTogglePanel(bool visible);
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
    DemoSource *m_demo = nullptr;

    QScrollArea *m_leftScroll = nullptr;   // wraps the controls panel
    QScrollArea *m_plotScroll = nullptr;   // wraps the stacked charts
    QAction *m_togglePanelAction = nullptr;

    QSpinBox *m_baudSpin = nullptr;
    QPushButton *m_connectButton = nullptr;
    QCheckBox *m_demoCheck = nullptr;
    QDoubleSpinBox *m_windowSpin = nullptr;
    QLineEdit *m_commandEdit = nullptr;
    QLabel *m_statusLabel = nullptr;
    QPlainTextEdit *m_logView = nullptr;
};
