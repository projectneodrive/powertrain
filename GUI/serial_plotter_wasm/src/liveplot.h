// Reusable stack of five scrolling charts (Target/Iq/Vel/Pos/Vbus), used by
// both the Live Plotter page and the PID Tuner page.
//
// Drawn with plain QPainter rather than Qt Charts: for five line traces at
// ~10 Hz the charts module is pure overhead (megabytes of WASM, heavier
// rendering) and, being a QGraphicsView, it also swallowed wheel events. Plain
// QWidgets keep the binary small and let the wheel reach the scroll area.
#pragma once

#include <QScrollArea>
#include <array>
#include <deque>

#include "channels.h"

class PlotStripe;

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

    struct Sample {
        double t;                                  // seconds relative to t0
        std::array<double, kNumChannels> v;
    };

private:
    double m_windowS = 20.0;
    bool m_haveT0 = false;
    double m_t0 = 0.0;
    std::deque<Sample> m_samples;
    std::array<PlotStripe *, kNumChannels> m_stripes{};
};
