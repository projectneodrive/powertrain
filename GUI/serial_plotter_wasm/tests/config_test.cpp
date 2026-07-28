// Checks that read-only config rows get a palette-appropriate tint: a light
// overlay in dark mode (the original bug: a fixed dark tint was invisible on a
// dark background) and a dark overlay in light mode.
#include "configpage.h"

#include <QApplication>
#include <QPalette>
#include <QTableWidget>
#include <QTableWidgetItem>

#include <cstdio>

static int failures = 0;
static void check(bool cond, const char *what)
{
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond)
        ++failures;
}

// Count read-only value cells whose tint matches a predicate on the red channel.
template <typename Pred>
static int countTinted(Pred pred)
{
    ConfigPage page;
    auto *table = page.findChild<QTableWidget *>();
    if (!table)
        return -1;
    int n = 0;
    for (int r = 0; r < table->rowCount(); ++r) {
        QTableWidgetItem *it = table->item(r, 1);   // value column
        if (it && it->background().style() != Qt::NoBrush) {
            const QColor c = it->background().color();
            if (c.alpha() > 0 && pred(c))
                ++n;
        }
    }
    return n;
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    {
        QPalette p;
        p.setColor(QPalette::Base, QColor(30, 30, 30));   // dark
        app.setPalette(p);
        const int light = countTinted([](const QColor &c) { return c.red() > 200; });
        std::printf("dark mode: %d light-tinted read-only cells\n", light);
        check(light >= 5, "dark mode tints read-only rows with a LIGHT overlay");
    }
    {
        QPalette p;
        p.setColor(QPalette::Base, QColor(255, 255, 255)); // light
        app.setPalette(p);
        const int dark = countTinted([](const QColor &c) { return c.red() < 60; });
        std::printf("light mode: %d dark-tinted read-only cells\n", dark);
        check(dark >= 5, "light mode tints read-only rows with a DARK overlay");
    }

    std::printf(failures ? "\nFAILED (%d)\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
