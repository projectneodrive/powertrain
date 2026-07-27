#include "liveplot.h"

#include <QPainter>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kMaxHistoryS = 300.0;   // hard cap on retained samples
constexpr int kMinPlotHeight = 200;      // per-chart floor before scrolling
}

// One channel's strip: a plain QWidget that paints its trace from the shared
// sample buffer owned by the LivePlot. Being a plain widget (not a
// QGraphicsView) it ignores wheel events, so they bubble to the scroll area.
class PlotStripe : public QWidget
{
public:
    PlotStripe(const std::deque<LivePlot::Sample> &samples, int channel,
               const double &windowS, bool bottom, QWidget *parent = nullptr)
        : QWidget(parent), m_samples(samples), m_channel(channel),
          m_windowS(windowS), m_bottom(bottom)
    {
        m_color = QColor(QString::fromLatin1(kChannels[channel].color));
        m_label = QString::fromLatin1(kChannels[channel].label);
        setMinimumHeight(kMinPlotHeight);
        setAttribute(Qt::WA_OpaquePaintEvent);   // we fill the whole rect
    }

protected:
    void paintEvent(QPaintEvent *) override;

private:
    const std::deque<LivePlot::Sample> &m_samples;
    int m_channel;
    const double &m_windowS;
    bool m_bottom;
    QColor m_color;
    QString m_label;
};

void PlotStripe::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.fillRect(rect(), Qt::white);

    const int left = 52;
    const int right = 8;
    const int top = 6;
    const int bottom = m_bottom ? 22 : 6;
    const QRectF area(left, top, std::max(10, width() - left - right),
                      std::max(10, height() - top - bottom));

    const QColor grid(230, 230, 230);
    const QColor axisText(90, 90, 90);
    p.setPen(grid);
    p.drawRect(area);

    // Label (top-left, inside).
    p.setPen(axisText);
    p.drawText(QPointF(left + 4, top + 14), m_label);

    if (m_samples.empty())
        return;

    const double tLast = m_samples.back().t;
    const double tStart = tLast - m_windowS;
    const double xLo = std::max(0.0, tStart);
    const double xHi = std::max(m_windowS, tLast);
    const double xSpan = std::max(1e-9, xHi - xLo);

    // Visible window (samples are time-ordered).
    auto it = std::lower_bound(m_samples.begin(), m_samples.end(), tStart,
                               [](const LivePlot::Sample &s, double v) { return s.t < v; });

    double yMin = std::numeric_limits<double>::max();
    double yMax = std::numeric_limits<double>::lowest();
    for (auto s = it; s != m_samples.end(); ++s) {
        const double y = s->v[m_channel];
        yMin = std::min(yMin, y);
        yMax = std::max(yMax, y);
    }
    if (yMin > yMax) { yMin = -1.0; yMax = 1.0; }
    double pad = (yMax - yMin) * 0.1;
    if (pad < 1e-6)
        pad = std::max(0.5, std::abs(yMax) * 0.1);
    yMin -= pad;
    yMax += pad;
    const double ySpan = std::max(1e-9, yMax - yMin);

    auto toPx = [&](double t, double y) {
        return QPointF(area.left() + (t - xLo) / xSpan * area.width(),
                       area.bottom() - (y - yMin) / ySpan * area.height());
    };

    // Y grid + labels at min / mid / max.
    p.setPen(axisText);
    const double yTicks[3] = {yMax, (yMin + yMax) * 0.5, yMin};
    for (double yv : yTicks) {
        const double py = toPx(xLo, yv).y();
        p.setPen(grid);
        p.drawLine(QPointF(area.left(), py), QPointF(area.right(), py));
        p.setPen(axisText);
        p.drawText(QRectF(0, py - 8, left - 6, 16),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(yv, 'g', 3));
    }

    if (m_bottom) {
        p.setPen(axisText);
        p.drawText(QRectF(area.left(), area.bottom() + 4, area.width(), 16),
                   Qt::AlignLeft, QString::number(xLo, 'f', 0));
        p.drawText(QRectF(area.left(), area.bottom() + 4, area.width(), 16),
                   Qt::AlignRight, QString::number(xHi, 'f', 0) + QStringLiteral(" s"));
    }

    // The trace.
    QList<QPointF> pts;
    pts.reserve(int(m_samples.end() - it));
    for (auto s = it; s != m_samples.end(); ++s)
        pts.append(toPx(s->t, s->v[m_channel]));
    if (pts.size() >= 2) {
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(m_color, 1.5));
        p.drawPolyline(pts.constData(), int(pts.size()));
    }
}

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
        m_stripes[ch] = new PlotStripe(m_samples, ch, m_windowS,
                                       ch == kNumChannels - 1);
        layout->addWidget(m_stripes[ch]);
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

    for (PlotStripe *s : m_stripes)
        s->update();
}

void LivePlot::clear()
{
    m_samples.clear();
    m_haveT0 = false;
    for (PlotStripe *s : m_stripes)
        s->update();
}

void LivePlot::setWindow(double seconds)
{
    m_windowS = seconds;
    for (PlotStripe *s : m_stripes)
        s->update();
}
