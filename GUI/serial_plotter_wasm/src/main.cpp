#include "plotwindow.h"

#include <QApplication>
#include <QCommandLineParser>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("serial_plotter_wasm"));

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("Live plotter for the powertrain firmware (Qt / WebAssembly)."));
    parser.addHelpOption();
    QCommandLineOption demoOption(
        QStringLiteral("demo"),
        QStringLiteral("Start with synthetic telemetry instead of a serial port."));
    QCommandLineOption windowOption(
        QStringLiteral("window"),
        QStringLiteral("Time window to display, in seconds (default: 20)."),
        QStringLiteral("seconds"), QStringLiteral("20"));
    parser.addOption(demoOption);
    parser.addOption(windowOption);
    parser.process(app);

    bool ok = false;
    const double windowS = parser.value(windowOption).toDouble(&ok);

    PlotWindow window(ok && windowS > 0.0 ? windowS : 20.0);
    window.show();
    if (parser.isSet(demoOption))
        window.enableDemo(true);
    if (qEnvironmentVariableIsSet("PLOTTER_SELFTEST")) {
        window.resize(1000, 600);
        QCoreApplication::processEvents();
        window.dumpScrollDiagnostics();
        return 0;
    }
    return app.exec();
}
