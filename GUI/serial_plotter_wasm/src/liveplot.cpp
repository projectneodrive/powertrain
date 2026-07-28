#include "liveplot.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace {

constexpr double kMaxHistoryS = 300.0;   // hard cap on retained samples
// Deliberately small so all five charts fit a normal window; the QScrollArea
// only scrolls once the viewport drops below 5x this.
constexpr int kMinChartHeight = 96;
constexpr int kHeaderHeight = 20;

// QChartView derives from QGraphicsView, which accepts wheel events even when
// it has nothing of its own to scroll -- that stops the enclosing QScrollArea
// from seeing them. We forward to the parent scroll area's scrollbar instead.
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
    m_rowLayout = new QVBoxLayout(container);
    m_rowLayout->setContentsMargins(0, 0, 0, 0);
    m_rowLayout->setSpacing(3);

    for (int ch = 0; ch < kNumChannels; ++ch) {
        auto *series = new QLineSeries;
        series->setColor(QColor(QString::fromLatin1(kChannels[ch].color)));

        auto *chart = new QChart;
        chart->legend()->hide();
        chart->addSeries(series);
        chart->setMargins(QMargins(2, 1, 6, 1));

        auto *axX = new QValueAxis;
        axX->setRange(0.0, m_windowS);
        axX->setLabelFormat(QStringLiteral("%.0f"));
        auto *axY = new QValueAxis;
        axY->setRange(-1.0, 1.0);

        chart->addAxis(axX, Qt::AlignBottom);
        chart->addAxis(axY, Qt::AlignLeft);
        series->attachAxis(axX);
        series->attachAxis(axY);

        auto *view = new ChartView(chart);
        view->setRenderHint(QPainter::Antialiasing, false);
        view->setMinimumHeight(kMinChartHeight);
        view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // Header = colour swatch + label, doubling as a drag handle.
        auto *header = new QLabel(
            QStringLiteral("⠿  %1").arg(QString::fromLatin1(kChannels[ch].label)));
        header->setFixedHeight(kHeaderHeight);
        header->setCursor(Qt::OpenHandCursor);
        header->setToolTip(QStringLiteral("Drag to reorder"));
        header->setStyleSheet(
            QStringLiteral("QLabel{background:#f0f0f0;border:1px solid #ddd;"
                           "padding-left:6px;color:%1;font-weight:bold;}")
                .arg(QString::fromLatin1(kChannels[ch].color)));
        header->installEventFilter(this);

        auto *rowWidget = new QWidget;
        auto *rowVLayout = new QVBoxLayout(rowWidget);
        rowVLayout->setContentsMargins(0, 0, 0, 0);
        rowVLayout->setSpacing(0);
        rowVLayout->addWidget(header);
        rowVLayout->addWidget(view, 1);
        rowWidget->setMinimumHeight(kMinChartHeight + kHeaderHeight);

        m_rowLayout->addWidget(rowWidget, 1);

        m_series[ch] = series;
        m_axX[ch] = axX;
        m_axY[ch] = axY;
        m_rows.push_back({rowWidget, header, ch});
    }

    setWidget(container);
    refreshBottomAxis();
}

int LivePlot::rowIndexOfHeader(const QObject *header) const
{
    for (size_t i = 0; i < m_rows.size(); ++i)
        if (m_rows[i].header == header)
            return int(i);
    return -1;
}

void LivePlot::refreshBottomAxis()
{
    // Only the bottom-most chart shows the time axis labels/title.
    for (size_t i = 0; i < m_rows.size(); ++i) {
        const bool bottom = (i == m_rows.size() - 1);
        QValueAxis *axX = m_axX[m_rows[i].channel];
        axX->setLabelsVisible(bottom);
        axX->setTitleText(bottom ? QStringLiteral("Time [s]") : QString());
    }
}

void LivePlot::moveRow(int from, int to)
{
    const int n = int(m_rows.size());
    if (from == to || from < 0 || to < 0 || from >= n || to >= n)
        return;

    Row moved = m_rows[from];
    m_rows.erase(m_rows.begin() + from);
    m_rows.insert(m_rows.begin() + to, moved);

    for (const Row &r : m_rows)
        m_rowLayout->removeWidget(r.container);
    for (const Row &r : m_rows)
        m_rowLayout->addWidget(r.container, 1);

    refreshBottomAxis();
}

bool LivePlot::eventFilter(QObject *obj, QEvent *ev)
{
    const int row = rowIndexOfHeader(obj);
    if (row < 0)
        return QScrollArea::eventFilter(obj, ev);

    switch (ev->type()) {
    case QEvent::MouseButtonPress: {
        auto *me = static_cast<QMouseEvent *>(ev);
        if (me->button() == Qt::LeftButton) {
            m_dragRow = row;
            m_dragging = false;
            m_pressY = me->globalPosition().toPoint().y();
        }
        return false;
    }
    case QEvent::MouseMove: {
        auto *me = static_cast<QMouseEvent *>(ev);
        if (m_dragRow < 0 || !(me->buttons() & Qt::LeftButton))
            return false;
        const int gy = me->globalPosition().toPoint().y();
        if (!m_dragging && std::abs(gy - m_pressY) > 6) {
            m_dragging = true;
            m_rows[m_dragRow].header->setCursor(Qt::ClosedHandCursor);
        }
        if (m_dragging) {
            for (size_t i = 0; i < m_rows.size(); ++i) {
                QWidget *c = m_rows[i].container;
                const int top = c->mapToGlobal(QPoint(0, 0)).y();
                if (gy >= top && gy < top + c->height()) {
                    if (int(i) != m_dragRow) {
                        moveRow(m_dragRow, int(i));
                        m_dragRow = int(i);
                    }
                    break;
                }
            }
        }
        return false;
    }
    case QEvent::MouseButtonRelease: {
        if (m_dragRow >= 0) {
            m_rows[m_dragRow].header->setCursor(Qt::OpenHandCursor);
            m_dragRow = -1;
            m_dragging = false;
        }
        return false;
    }
    default:
        return QScrollArea::eventFilter(obj, ev);
    }
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
