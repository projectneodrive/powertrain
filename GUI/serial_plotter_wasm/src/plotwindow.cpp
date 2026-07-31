#include "plotwindow.h"
#include "cheatsheetpage.h"
#include "configpage.h"
#include "demosource.h"
#include "mainview.h"
#include "serialbridge.h"
#include "telemetryhub.h"

#include <QActionGroup>
#include <QCheckBox>
#include <QKeySequence>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QToolBar>
#include <QWidget>

PlotWindow::PlotWindow(double windowS, QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Powertrain console (WASM)"));
    resize(1320, 880);

    // Telemetry singleton must exist before the views subscribe to it.
    TelemetryHub::instance();

    m_stack = new QStackedWidget(this);
    setCentralWidget(m_stack);
    m_mainView = new MainView(windowS);
    m_configPage = new ConfigPage();
    m_cheatPage = new CheatSheetPage();
    m_stack->addWidget(m_mainView);
    m_stack->addWidget(m_configPage);
    m_stack->addWidget(m_cheatPage);

    // ----------------------------------------------------------- toolbar --
    auto *toolbar = addToolBar(QStringLiteral("Main"));
    toolbar->setMovable(false);

    // Page selector: checkable toolbar buttons (no QMenu popup -- those are
    // unstable in Qt for WebAssembly).
    auto *pageGroup = new QActionGroup(this);
    pageGroup->setExclusive(true);
    auto addPage = [&](const QString &name) {
        QAction *a = toolbar->addAction(name);
        a->setCheckable(true);
        pageGroup->addAction(a);
        return a;
    };
    QAction *plotterAct = addPage(QStringLiteral("Live Plotter"));
    QAction *tunerAct = addPage(QStringLiteral("PID Tuner"));
    QAction *configAct = addPage(QStringLiteral("Motor Config"));
    QAction *cmdAct = addPage(QStringLiteral("Commands"));
    plotterAct->setChecked(true);
    connect(plotterAct, &QAction::triggered, this, [this] { showMainPanel(MainView::PlotterPanel); });
    connect(tunerAct, &QAction::triggered, this, [this] { showMainPanel(MainView::TunerPanel); });
    connect(configAct, &QAction::triggered, this, [this] { showPage(m_configPage); });
    connect(cmdAct, &QAction::triggered, this, [this] { showPage(m_cheatPage); });

    toolbar->addSeparator();

    // Global USB connection (shared by all views).
    m_baudSpin = new QSpinBox;
    m_baudSpin->setRange(300, 4000000);
    m_baudSpin->setValue(115200);
    m_baudSpin->setToolTip(QStringLiteral("Baud (ignored by USB CDC, but Web Serial needs a value)"));
    m_connectButton = new QPushButton(QStringLiteral("Connect (USB)"));
    m_connectButton->setToolTip(QStringLiteral("Connect / disconnect the board (F2 or Ctrl+K)"));

    // Raccourci connexion/déconnexion. Deux séquences : F2 passe partout, alors
    // que Ctrl+K est intercepté par la barre d'adresse de certains navigateurs
    // -- en WASM on ne peut pas le reprendre, d'où le doublon.
    // ApplicationShortcut : marche quelle que soit la page affichée, y compris
    // quand le focus est dans un champ de saisie.
    m_connectAction = new QAction(QStringLiteral("Connect / Disconnect"), this);
    m_connectAction->setShortcuts({QKeySequence(Qt::Key_F2),
                                   QKeySequence(QStringLiteral("Ctrl+K"))});
    m_connectAction->setShortcutContext(Qt::ApplicationShortcut);
    connect(m_connectAction, &QAction::triggered, this, &PlotWindow::onConnectClicked);
    addAction(m_connectAction);

    m_demoCheck = new QCheckBox(QStringLiteral("Demo"));
    m_demoCheck->setToolTip(QStringLiteral("Synthetic telemetry, no hardware"));
    toolbar->addWidget(new QLabel(QStringLiteral(" Baud ")));
    toolbar->addWidget(m_baudSpin);
    toolbar->addWidget(m_connectButton);
    toolbar->addWidget(m_demoCheck);

    toolbar->addSeparator();

    m_togglePanelAction = toolbar->addAction(QStringLiteral("Hide panel"));
    m_togglePanelAction->setCheckable(true);
    m_togglePanelAction->setChecked(true);
    m_togglePanelAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+B")));
    m_togglePanelAction->setToolTip(QStringLiteral("Show/hide the control panel (Ctrl+B)"));

    auto *spacer = new QWidget;
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    toolbar->addWidget(spacer);
    m_statusLabel = new QLabel(QStringLiteral("Disconnected — build %1 %2 ")
                                   .arg(QString::fromLatin1(__DATE__),
                                        QString::fromLatin1(__TIME__)));
    toolbar->addWidget(m_statusLabel);

    // --------------------------------------------------------- wiring --
    m_demo = new DemoSource(this);
    connect(m_connectButton, &QPushButton::clicked, this, &PlotWindow::onConnectClicked);
    connect(m_demoCheck, &QCheckBox::toggled, this, &PlotWindow::onDemoToggled);
    connect(m_togglePanelAction, &QAction::toggled, this, &PlotWindow::onTogglePanel);
    connect(&SerialBridge::instance(), &SerialBridge::statusChanged,
            this, &PlotWindow::onStatusChanged);
}

void PlotWindow::showMainPanel(int panel)
{
    m_stack->setCurrentWidget(m_mainView);
    m_mainView->showPanel(static_cast<MainView::Panel>(panel));
    m_togglePanelAction->setEnabled(true);
    m_mainView->setSidePanelVisible(m_togglePanelAction->isChecked());
}

void PlotWindow::showPage(QWidget *page)
{
    m_stack->setCurrentWidget(page);
    m_togglePanelAction->setEnabled(false);   // full-view pages have no side panel
}

void PlotWindow::enableDemo(bool on)
{
    m_demoCheck->setChecked(on);
}

void PlotWindow::onConnectClicked()
{
    auto &bridge = SerialBridge::instance();
    if (bridge.isConnected())
        bridge.disconnectPort();
    else
        bridge.connectPort(m_baudSpin->value());
}

void PlotWindow::onDemoToggled(bool on)
{
    if (on)
        m_demo->start();
    else
        m_demo->stop();
    m_connectButton->setEnabled(!on);
    m_connectAction->setEnabled(!on);   // le raccourci suit l'état du bouton
    m_statusLabel->setText(on ? QStringLiteral("Demo mode — synthetic telemetry ")
                              : QStringLiteral("Demo stopped "));
}

void PlotWindow::onStatusChanged(bool connected, const QString &message)
{
    m_connectButton->setText(connected ? QStringLiteral("Disconnect")
                                       : QStringLiteral("Connect (USB)"));
    m_connectAction->setText(connected ? QStringLiteral("Disconnect")
                                       : QStringLiteral("Connect / Disconnect"));
    m_baudSpin->setEnabled(!connected);
    m_demoCheck->setEnabled(!connected);
    m_statusLabel->setText(message + QLatin1Char(' '));
}

void PlotWindow::onTogglePanel(bool visible)
{
    m_togglePanelAction->setText(visible ? QStringLiteral("Hide panel")
                                         : QStringLiteral("Show panel"));
    if (m_stack->currentWidget() == m_mainView)
        m_mainView->setSidePanelVisible(visible);
}
