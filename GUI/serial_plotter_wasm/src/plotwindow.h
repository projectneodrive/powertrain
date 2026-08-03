// Application shell: a top toolbar (page buttons, global USB connection, panel
// toggle, connection status) over a QStackedWidget with two entries -- the
// combined plotter/tuner MainView and the Motor Config page.
//
// The Live Plotter and PID Tuner buttons both show the MainView and only swap
// its left control panel; the Motor Config button swaps the whole central view.
#pragma once

#include <QMainWindow>

QT_BEGIN_NAMESPACE
class QAction;
class QCheckBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QStackedWidget;
QT_END_NAMESPACE

class CanDevicesPage;
class CheatSheetPage;
class ConfigPage;
class DemoSource;
class MainView;

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

private:
    void showMainPanel(int panel);   // MainView::Panel
    void showPage(QWidget *page);    // a full-view page (Config / Commands)

    QStackedWidget *m_stack = nullptr;
    MainView *m_mainView = nullptr;
    ConfigPage *m_configPage = nullptr;
    CanDevicesPage *m_canPage = nullptr;
    QWidget *m_canPageHost = nullptr;   // the QScrollArea m_canPage lives in
    CheatSheetPage *m_cheatPage = nullptr;
    DemoSource *m_demo = nullptr;

    QSpinBox *m_baudSpin = nullptr;
    QCheckBox *m_demoCheck = nullptr;
    QPushButton *m_connectButton = nullptr;
    QAction *m_connectAction = nullptr;      // porte le raccourci clavier
    QAction *m_togglePanelAction = nullptr;
    QLabel *m_statusLabel = nullptr;
};
