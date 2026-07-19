#include "plotwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    PlotWindow window(20.0);
    window.show();
    return app.exec();
}
