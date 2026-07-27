// Application shell: a top toolbar (pages menu, global USB connection, panel
// toggle, connection status) over a QStackedWidget holding the three pages.
// Serial I/O and telemetry are global (SerialBridge / TelemetryHub singletons),
// so every page shares one connection.
#pragma once

#include <QList>
#include <QMainWindow>

QT_BEGIN_NAMESPACE
class QAction;
class QCheckBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QStackedWidget;
QT_END_NAMESPACE

class AppPage;
class DemoSource;

class PlotWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit PlotWindow(double windowS, QWidget *parent = nullptr);

    // Start with synthetic telemetry (the --demo command line switch).
    void enableDemo(bool on);

private slots:
    void onConnectClicked();
    void onDemoToggled(bool on);
    void onStatusChanged(bool connected, const QString &message);
    void onTogglePanel(bool visible);
    void onPageChanged(int index);

private:
    QStackedWidget *m_stack = nullptr;
    QList<AppPage *> m_pages;
    DemoSource *m_demo = nullptr;

    QSpinBox *m_baudSpin = nullptr;
    QCheckBox *m_demoCheck = nullptr;
    QPushButton *m_connectButton = nullptr;
    QAction *m_togglePanelAction = nullptr;
    QLabel *m_statusLabel = nullptr;
};
