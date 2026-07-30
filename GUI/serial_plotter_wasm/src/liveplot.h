// Reusable stack of scrolling charts, one per telemetry channel (see
// channels.h: Target/Iq/Vel/Pos/Vbus + sensorless obsΔV/blend).
//
// Layout: compact by design -- the charts share the available height and all
// five fit on screen when the window is reasonably tall; only when the viewport
// gets short does the QScrollArea start scrolling. Each chart has a small
// header that doubles as a drag handle to reorder the charts.
#pragma once

#include <QScrollArea>
#include <array>
#include <deque>
#include <vector>

#include "channels.h"

QT_BEGIN_NAMESPACE
class QChart;
class QLineSeries;
class QSplitter;
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

    // Show/hide an individual channel's chart (driven by the plotter panel's
    // checkboxes). Hiding a chart removes its row from the splitter and hands
    // its height to the rest.
    void setChannelVisible(int channel, bool visible);
    bool isChannelVisible(int channel) const;

protected:
    bool eventFilter(QObject *obj, QEvent *ev) override;

private:
    void redraw();
    void moveRow(int from, int to);
    int rowIndexOfHeader(const QObject *header) const;
    void updateSplitterMinHeight();   // size for the currently visible rows

    struct Sample {
        double t;                                  // seconds relative to t0
        std::array<double, kNumChannels> v;
    };
    struct Row {
        QWidget *container;
        QWidget *header;
        int channel;                               // which channel it displays
    };

    double m_windowS = 20.0;
    bool m_haveT0 = false;
    double m_t0 = 0.0;
    std::deque<Sample> m_samples;

    QSplitter *m_splitter = nullptr;               // holds the resizable rows
    std::vector<Row> m_rows;                        // in visual (top-to-bottom) order

    // Series/axes indexed by channel (fixed; reordering only moves widgets).
    std::array<QLineSeries *, kNumChannels> m_series{};
    std::array<QValueAxis *, kNumChannels> m_axX{};
    std::array<QValueAxis *, kNumChannels> m_axY{};

    // Drag-reorder state.
    int m_dragRow = -1;
    bool m_dragging = false;
    int m_pressY = 0;
};
