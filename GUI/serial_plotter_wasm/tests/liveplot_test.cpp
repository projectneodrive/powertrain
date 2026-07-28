// Verifies the two riskiest LivePlot behaviours headlessly:
//   1. Compact layout: all charts fit a tall viewport (no scroll), and it
//      starts scrolling when the viewport is short.
//   2. Drag-to-reorder: dragging the top chart's header down past the others
//      actually moves that row down.
#include "liveplot.h"

#include <QApplication>
#include <QImage>
#include <QLabel>
#include <QMouseEvent>
#include <QScrollBar>
#include <QSplitter>

#include <algorithm>
#include <array>
#include <cmath>
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

    std::printf("Axis scale labels:\n");
    plot.resize(800, 1000);
    for (int i = 0; i < 200; ++i) {
        const double t = i * 0.1;
        std::array<double, kNumChannels> v{};
        for (int c = 0; c < kNumChannels; ++c)
            v[c] = std::sin(t + c) * (c + 3);        // distinct ranges per chart
        plot.addSample(t, v);
    }
    app.processEvents();
    {
        QWidget *content = plot.widget();
        const QImage img = content->grab().toImage();   // grab() forces a paint
        if (qEnvironmentVariableIsSet("DUMP_PNG"))
            img.save(qEnvironmentVariable("DUMP_PNG"));
        int nonWhite = 0;
        for (int y = 0; y < img.height(); y += 2)
            for (int x = 0; x < img.width(); x += 2) {
                const QRgb px = img.pixel(x, y);
                if (qRed(px) < 245 || qGreen(px) < 245 || qBlue(px) < 245)
                    ++nonWhite;
            }
        std::printf("  (chart renders offscreen? non-white pixels: %d)\n", nonWhite);
        // Y tick labels sit in the left margin, left of the plot area (which
        // QtCharts positions at ~x=160 once it has reserved label space). If
        // the labels were clipped (the original bug) this column would be
        // blank. Header text also lives on the left but only in the top ~20 px
        // of each row, which we skip.
        // NB: under the offscreen platform the glyphs render as tofu boxes for
        // lack of a font, but they are still dark pixels in the right place --
        // enough to prove the labels are laid out and not clipped.
        auto hs2 = headers(&plot);
        int labelPixels = 0;
        for (int y = 0; y < img.height(); ++y) {
            bool inHeader = false;
            for (QLabel *h : hs2) {
                const int hy = h->mapTo(content, QPoint(0, 0)).y();
                if (y >= hy - 2 && y <= hy + h->height() + 1) { inHeader = true; break; }
            }
            if (inHeader)
                continue;
            for (int x = 55; x < 158; ++x) {
                const QRgb px = img.pixel(x, y);
                if (qRed(px) < 170 && qGreen(px) < 170 && qBlue(px) < 170)
                    ++labelPixels;
            }
        }
        std::printf("  (dark pixels in the Y-label margin: %d)\n", labelPixels);
        check(labelPixels > 60, "Y-axis scale labels are rendered (not clipped)");
    }

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

    std::printf("Resizable graphs:\n");
    {
        auto *sp = qobject_cast<QSplitter *>(plot.widget());
        check(sp && sp->count() == 5, "charts live in a 5-way vertical splitter");
        if (sp) {
            // A small, scrolling viewport -- the case where the old fixed-min
            // layout had zero slack and dragging did nothing.
            plot.resize(800, 600);
            app.processEvents();
            sp->setSizes({500, 100, 100, 100, 100});   // make the top graph tall
            app.processEvents();
            const QList<int> sizes = sp->sizes();
            std::printf("  (splitter sizes: %d %d %d %d %d)\n",
                        sizes.value(0), sizes.value(1), sizes.value(2),
                        sizes.value(3), sizes.value(4));
            check(sizes.size() == 5 && sizes[0] > sizes[1] + 200,
                  "a graph resizes even in a short, scrolling viewport");
        }
    }

    std::printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
