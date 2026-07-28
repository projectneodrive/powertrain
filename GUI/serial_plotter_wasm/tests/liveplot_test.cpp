// Verifies the two riskiest LivePlot behaviours headlessly:
//   1. Compact layout: all charts fit a tall viewport (no scroll), and it
//      starts scrolling when the viewport is short.
//   2. Drag-to-reorder: dragging the top chart's header down past the others
//      actually moves that row down.
#include "liveplot.h"

#include <QApplication>
#include <QLabel>
#include <QMouseEvent>
#include <QScrollBar>

#include <algorithm>
#include <cstdio>
#include <vector>

static int failures = 0;
static void check(bool cond, const char *what)
{
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond)
        ++failures;
}

// Header drag handles are the QLabels whose text starts with the grip glyph.
static std::vector<QLabel *> headers(QWidget *root)
{
    std::vector<QLabel *> hs;
    for (QLabel *l : root->findChildren<QLabel *>())
        if (l->text().startsWith(QStringLiteral("⠿")))   // ⠿
            hs.push_back(l);
    std::sort(hs.begin(), hs.end(), [](QLabel *a, QLabel *b) {
        return a->mapToGlobal(QPoint(0, 0)).y() < b->mapToGlobal(QPoint(0, 0)).y();
    });
    return hs;
}

static void sendMouse(QWidget *w, QEvent::Type type, QPoint global, Qt::MouseButtons buttons)
{
    QMouseEvent ev(type, w->mapFromGlobal(global), QPointF(global),
                   Qt::LeftButton, buttons, Qt::NoModifier);
    QApplication::sendEvent(w, &ev);
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    LivePlot plot;

    std::printf("Compact layout:\n");
    plot.resize(800, 1000);           // tall
    plot.show();
    app.processEvents();
    check(plot.verticalScrollBar()->maximum() == 0, "all charts fit a tall viewport (no scroll)");

    plot.resize(800, 300);            // short
    app.processEvents();
    check(plot.verticalScrollBar()->maximum() > 0, "short viewport scrolls");

    std::printf("Drag reorder:\n");
    plot.resize(800, 1000);
    app.processEvents();
    auto hs = headers(&plot);
    check(hs.size() == 5, "found five draggable headers");

    if (hs.size() == 5) {
        QLabel *top = hs.front();
        const QString topText = top->text();
        QLabel *bottom = hs.back();

        QPoint from = top->mapToGlobal(QRect(QPoint(0, 0), top->size()).center());
        QPoint to = bottom->mapToGlobal(QRect(QPoint(0, 0), bottom->size()).center());

        sendMouse(top, QEvent::MouseButtonPress, from, Qt::LeftButton);
        // Step the cursor down so it crosses each row's global rect.
        for (int step = 1; step <= 8; ++step) {
            QPoint p(from.x(), from.y() + (to.y() - from.y()) * step / 8);
            sendMouse(top, QEvent::MouseMove, p, Qt::LeftButton);
            app.processEvents();
        }
        sendMouse(top, QEvent::MouseButtonRelease, to, Qt::NoButton);
        app.processEvents();

        auto after = headers(&plot);
        check(!after.empty() && after.back()->text() == topText,
              "dragged header is now the bottom row");
    }

    std::printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
