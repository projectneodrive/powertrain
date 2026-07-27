// Renders a LivePlot offscreen after feeding it data and confirms the traces
// actually paint (non-white pixels), catching a silently-blank custom plot.
#include "liveplot.h"

#include <QApplication>
#include <QImage>

#include <array>
#include <cmath>
#include <cstdio>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    LivePlot plot;
    plot.setWindow(20.0);
    plot.resize(600, 1100);

    for (int i = 0; i < 200; ++i) {
        const double t = i * 0.1;
        std::array<double, kNumChannels> v{};
        for (int c = 0; c < kNumChannels; ++c)
            v[c] = std::sin(t + c) * (c + 1);
        plot.addSample(t, v);
    }
    app.processEvents();

    QWidget *content = plot.widget();
    content->resize(600, 1100);
    QImage img(content->size(), QImage::Format_ARGB32);
    content->render(&img);

    int colored = 0;
    for (int y = 0; y < img.height(); y += 2)
        for (int x = 0; x < img.width(); x += 2) {
            const QRgb px = img.pixel(x, y);
            if (qRed(px) < 240 || qGreen(px) < 240 || qBlue(px) < 240)
                ++colored;
        }

    std::printf("content %dx%d, non-white sampled pixels = %d\n",
                img.width(), img.height(), colored);
    const bool ok = colored > 500;
    std::printf(ok ? "PASS: plot renders traces\n" : "FAIL: plot looks blank\n");
    return ok ? 0 : 1;
}
