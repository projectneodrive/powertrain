// Reusable stack of five scrolling charts (Target/Iq/Vel/Pos/Vbus), used by
// both the Live Plotter page and the PID Tuner page. Is itself a QScrollArea
// so the charts stay readable (fixed per-chart minimum height) and scroll when
// the viewport is short.
#pragma once

#include <QScrollArea>
#include <array>
#include <deque>

#include "channels.h"

QT_BEGIN_NAMESPACE
class QChart;
class QLineSeries;
class QValueAxis;
QT_END_NAMESPACE

class LivePlot : public QScrollArea
{
    Q_OBJECT
public:
    explicit LivePlot(QWidget *parent = nullptr);

    // tSec is an absolute firmware timestamp (seconds); the first sample sets
    // t0 and everything is drawn relative to it.
    void addSample(double tSec, const std::array<double, kNumChannels> &vals);
    void clear();
    void setWindow(double seconds);
    double window() const { return m_windowS; }

private:
    void redraw();

    struct Sample {
        double t;                                  // seconds relative to t0
        std::array<double, kNumChannels> v;
    };

    double m_windowS = 20.0;
    bool m_haveT0 = false;
    double m_t0 = 0.0;
    std::deque<Sample> m_samples;

    std::array<QChart *, kNumChannels> m_charts{};
    std::array<QLineSeries *, kNumChannels> m_series{};
    std::array<QValueAxis *, kNumChannels> m_axX{};
    std::array<QValueAxis *, kNumChannels> m_axY{};
};
