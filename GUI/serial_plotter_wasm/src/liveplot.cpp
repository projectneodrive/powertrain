#include "liveplot.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QPainter>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kMaxHistoryS = 300.0;   // hard cap on retained samples
constexpr int kMinPlotHeight = 200;      // per-chart floor before scrolling

// QChartView derives from QGraphicsView, which accepts wheel events even when
// it has nothing of its own to scroll -- that stops the enclosing QScrollArea
// from ever seeing them. ignore() alone is unreliable (more so under WASM), so
// we drive the parent scroll area's scrollbar directly.
class ChartView : public QChartView
{
public:
    explicit ChartView(QChart *chart) : QChartView(chart)
    {
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }

protected:
    void wheelEvent(QWheelEvent *event) override
    {
        for (QWidget *w = parentWidget(); w; w = w->parentWidget()) {
            if (auto *area = qobject_cast<QScrollArea *>(w)) {
                QScrollBar *bar = area->verticalScrollBar();
                bar->setValue(bar->value() - event->angleDelta().y());
                event->accept();
                return;
            }
        }
        event->ignore();
    }
};

} // namespace

LivePlot::LivePlot(QWidget *parent) : QScrollArea(parent)
{
    setWidgetResizable(true);
    setFrameShape(QFrame::NoFrame);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto *container = new QWidget;
    auto *layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);
    container->setMinimumHeight(kMinPlotHeight * kNumChannels);

    for (int ch = 0; ch < kNumChannels; ++ch) {
        auto *series = new QLineSeries;
        series->setColor(QColor(QString::fromLatin1(kChannels[ch].color)));
        series->setName(QString::fromLatin1(kChannels[ch].label));

        auto *chart = new QChart;
        chart->legend()->hide();
        chart->addSeries(series);
        chart->setMargins(QMargins(4, 2, 8, 2));

        auto *axX = new QValueAxis;
        axX->setRange(0.0, m_windowS);
        axX->setLabelFormat(QStringLiteral("%.0f"));
        auto *axY = new QValueAxis;
        axY->setTitleText(QString::fromLatin1(kChannels[ch].label));
        axY->setRange(-1.0, 1.0);

        chart->addAxis(axX, Qt::AlignBottom);
        chart->addAxis(axY, Qt::AlignLeft);
        series->attachAxis(axX);
        series->attachAxis(axY);
        axX->setLabelsVisible(ch == kNumChannels - 1);
        if (ch == kNumChannels - 1)
            axX->setTitleText(QStringLiteral("Time [s]"));

        auto *view = new ChartView(chart);
        view->setRenderHint(QPainter::Antialiasing, false);
        view->setMinimumHeight(kMinPlotHeight);
        layout->addWidget(view);

        m_series[ch] = series;
        m_charts[ch] = chart;
        m_axX[ch] = axX;
        m_axY[ch] = axY;
    }

    setWidget(container);
}

void LivePlot::addSample(double tSec, const std::array<double, kNumChannels> &vals)
{
    if (!m_haveT0) {
        m_t0 = tSec;
        m_haveT0 = true;
    }
    m_samples.push_back({tSec - m_t0, vals});

    const double cutoff = (tSec - m_t0) - kMaxHistoryS;
    while (!m_samples.empty() && m_samples.front().t < cutoff)
        m_samples.pop_front();

    redraw();
}

void LivePlot::redraw()
{
    if (m_samples.empty())
        return;

    const double tLast = m_samples.back().t;
    const double tStart = tLast - m_windowS;

    auto it = std::lower_bound(m_samples.begin(), m_samples.end(), tStart,
                               [](const Sample &s, double v) { return s.t < v; });

    std::array<QList<QPointF>, kNumChannels> points;
    std::array<double, kNumChannels> yMin;
    std::array<double, kNumChannels> yMax;
    yMin.fill(std::numeric_limits<double>::max());
    yMax.fill(std::numeric_limits<double>::lowest());

    for (auto s = it; s != m_samples.end(); ++s) {
        for (int ch = 0; ch < kNumChannels; ++ch) {
            const double y = s->v[ch];
            points[ch].append(QPointF(s->t, y));
            yMin[ch] = std::min(yMin[ch], y);
            yMax[ch] = std::max(yMax[ch], y);
        }
    }

    const double xLo = std::max(0.0, tStart);
    const double xHi = std::max(m_windowS, tLast);
    for (int ch = 0; ch < kNumChannels; ++ch) {
        m_series[ch]->replace(points[ch]);
        m_axX[ch]->setRange(xLo, xHi);
        double lo = yMin[ch];
        double hi = yMax[ch];
        if (lo > hi) { lo = -1.0; hi = 1.0; }        // no points in window
        double pad = (hi - lo) * 0.1;
        if (pad < 1e-6)
            pad = std::max(0.5, std::abs(hi) * 0.1);
        m_axY[ch]->setRange(lo - pad, hi + pad);
    }
}

void LivePlot::clear()
{
    m_samples.clear();
    m_haveT0 = false;
    for (int ch = 0; ch < kNumChannels; ++ch) {
        m_series[ch]->clear();
        m_axX[ch]->setRange(0.0, m_windowS);
        m_axY[ch]->setRange(-1.0, 1.0);
    }
}

void LivePlot::setWindow(double seconds)
{
    m_windowS = seconds;
    redraw();
}
