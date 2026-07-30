#include "liveplot.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QFont>
#include <QLabel>
#include <QMargins>
#include <QMouseEvent>
#include <QPainter>
#include <QScrollBar>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace {

constexpr double kMaxHistoryS = 300.0;   // hard cap on retained samples
constexpr int kHeaderHeight = 20;
// A chart can be dragged this small (leaves room for the axis labels) -- the
// low minimum is what gives the splitter slack to actually resize.
constexpr int kMinChartHeight = 80;
// Default per-chart height. The splitter's total is set to N x this so there is
// always give to trade between neighbours; when it exceeds the viewport the
// area scrolls (so all charts fit a tall screen and scroll on a short one).
constexpr int kNaturalChartHeight = 160;

// QChartView is a QGraphicsView, which swallows wheel events even with nothing
// to scroll. Forward them to the enclosing QScrollArea so the chart column
// scrolls as expected.
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

    // A vertical splitter makes each graph individually resizable: drag the
    // handle between two charts to change their heights. It still scrolls when
    // the viewport is shorter than the charts' combined minimum.
    m_splitter = new QSplitter(Qt::Vertical);
    m_splitter->setChildrenCollapsible(false);
    m_splitter->setHandleWidth(8);
    // Visible grab bar between charts so the drag target is obvious.
    m_splitter->setStyleSheet(QStringLiteral(
        "QSplitter::handle:vertical{background:#c8c8c8;margin:1px 0;}"
        "QSplitter::handle:vertical:hover{background:#3b82f6;}"));

    for (int ch = 0; ch < kNumChannels; ++ch) {
        auto *series = new QLineSeries;
        series->setColor(QColor(QString::fromLatin1(kChannels[ch].color)));

        auto *chart = new QChart;
        chart->legend()->hide();
        chart->addSeries(series);
        // Axis tick labels are drawn INSIDE these margins, so the left/bottom
        // must be wide enough or the scale gets clipped (the original bug). We
        // set them explicitly rather than trusting auto-sizing.
        chart->setMargins(QMargins(48, 3, 10, 22));

        QFont labelFont;
        labelFont.setPointSize(8);

        auto *axX = new QValueAxis;
        axX->setRange(0.0, m_windowS);
        axX->setLabelFormat(QStringLiteral("%.0f"));
        axX->setLabelsFont(labelFont);
        auto *axY = new QValueAxis;
        axY->setRange(-1.0, 1.0);
        axY->setTickCount(3);                 // min / mid / max -- compact
        axY->setLabelFormat(QStringLiteral("%.3g"));
        axY->setLabelsFont(labelFont);

        chart->addAxis(axX, Qt::AlignBottom);
        chart->addAxis(axY, Qt::AlignLeft);
        series->attachAxis(axX);
        series->attachAxis(axY);
        // Time labels on every chart (so the scale is always visible regardless
        // of drag-reorder), not just the bottom one.
        axX->setLabelsVisible(true);

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

        m_splitter->addWidget(rowWidget);

        m_series[ch] = series;
        m_axX[ch] = axX;
        m_axY[ch] = axY;
        m_rows.push_back({rowWidget, header, ch});
    }

    setWidget(m_splitter);
    updateSplitterMinHeight();
}

void LivePlot::updateSplitterMinHeight()
{
    // Natural total taller than a typical viewport -> the area scrolls and,
    // crucially, there's slack for the handles to redistribute height. Scale it
    // to the number of *visible* rows so hidden charts don't leave dead space.
    int visible = 0;
    for (const Row &r : m_rows)
        if (r.container->isVisibleTo(m_splitter))   // ignores whether we're shown
            ++visible;
    m_splitter->setMinimumHeight(
        (kNaturalChartHeight + kHeaderHeight) * std::max(1, visible));
}

void LivePlot::setChannelVisible(int channel, bool visible)
{
    for (const Row &r : m_rows) {
        if (r.channel == channel) {
            r.container->setVisible(visible);   // QSplitter hides its handle too
            break;
        }
    }
    updateSplitterMinHeight();
}

bool LivePlot::isChannelVisible(int channel) const
{
    for (const Row &r : m_rows)
        if (r.channel == channel)
            return r.container->isVisibleTo(m_splitter);
    return false;
}

int LivePlot::rowIndexOfHeader(const QObject *header) const
{
    for (size_t i = 0; i < m_rows.size(); ++i)
        if (m_rows[i].header == header)
            return int(i);
    return -1;
}

void LivePlot::moveRow(int from, int to)
{
    const int n = int(m_rows.size());
    if (from == to || from < 0 || to < 0 || from >= n || to >= n)
        return;

    Row moved = m_rows[from];
    m_rows.erase(m_rows.begin() + from);
    m_rows.insert(m_rows.begin() + to, moved);

    // insertWidget on an existing child moves it to the new position.
    m_splitter->insertWidget(to, moved.container);
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
