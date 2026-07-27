#include "plotwindow.h"
#include "apppage.h"
#include "configpage.h"
#include "demosource.h"
#include "pidtunerpage.h"
#include "plotterpage.h"
#include "serialbridge.h"
#include "telemetryhub.h"

#include <QActionGroup>
#include <QCheckBox>
#include <QKeySequence>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QToolBar>
#include <QToolButton>
#include <QWidget>

PlotWindow::PlotWindow(double windowS, QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("Powertrain console (WASM)"));
    resize(1320, 880);

    // Telemetry singleton must exist before the pages subscribe to it.
    TelemetryHub::instance();

    m_stack = new QStackedWidget(this);
    setCentralWidget(m_stack);

    m_pages = {
        new PlotterPage(windowS),
        new ConfigPage(),
        new PidTunerPage(windowS),
    };
    for (AppPage *page : m_pages)
        m_stack->addWidget(page);

    // ----------------------------------------------------------- toolbar --
    auto *toolbar = addToolBar(QStringLiteral("Main"));
    toolbar->setMovable(false);

    // Pages menu (the "menu in the top bar" that switches pages).
    auto *pagesButton = new QToolButton;
    pagesButton->setText(QStringLiteral("☰ Pages"));
    pagesButton->setPopupMode(QToolButton::InstantPopup);
    auto *pagesMenu = new QMenu(pagesButton);
    auto *pageGroup = new QActionGroup(this);
    for (int i = 0; i < m_pages.size(); ++i) {
        QAction *a = pagesMenu->addAction(m_pages.at(i)->pageTitle());
        a->setCheckable(true);
        a->setChecked(i == 0);
        pageGroup->addAction(a);
        connect(a, &QAction::triggered, this, [this, i] { m_stack->setCurrentIndex(i); });
    }
    pagesButton->setMenu(pagesMenu);
    toolbar->addWidget(pagesButton);

    toolbar->addSeparator();

    // Global USB connection (shared by all pages).
    m_baudSpin = new QSpinBox;
    m_baudSpin->setRange(300, 4000000);
    m_baudSpin->setValue(115200);
    m_baudSpin->setToolTip(QStringLiteral("Baud (ignored by USB CDC, but Web Serial needs a value)"));
    m_connectButton = new QPushButton(QStringLiteral("Connect (USB)"));
    m_demoCheck = new QCheckBox(QStringLiteral("Demo"));
    m_demoCheck->setToolTip(QStringLiteral("Synthetic telemetry, no hardware"));
    toolbar->addWidget(new QLabel(QStringLiteral(" Baud ")));
    toolbar->addWidget(m_baudSpin);
    toolbar->addWidget(m_connectButton);
    toolbar->addWidget(m_demoCheck);

    toolbar->addSeparator();

    // Side-panel toggle (applies to whichever page has a panel).
    m_togglePanelAction = toolbar->addAction(QStringLiteral("Hide panel"));
    m_togglePanelAction->setCheckable(true);
    m_togglePanelAction->setChecked(true);
    m_togglePanelAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+B")));
    m_togglePanelAction->setToolTip(QStringLiteral("Show/hide the side panel (Ctrl+B)"));

    // Right-aligned connection status + build stamp (a stale cached .wasm is
    // then obvious at a glance).
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
    connect(m_stack, &QStackedWidget::currentChanged, this, &PlotWindow::onPageChanged);
    connect(&SerialBridge::instance(), &SerialBridge::statusChanged,
            this, &PlotWindow::onStatusChanged);

    onPageChanged(0);
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
    // The real port and the generator would otherwise interleave.
    m_connectButton->setEnabled(!on);
    m_statusLabel->setText(on ? QStringLiteral("Demo mode — synthetic telemetry ")
                              : QStringLiteral("Demo stopped "));
}

void PlotWindow::onStatusChanged(bool connected, const QString &message)
{
    m_connectButton->setText(connected ? QStringLiteral("Disconnect")
                                       : QStringLiteral("Connect (USB)"));
    m_baudSpin->setEnabled(!connected);
    m_demoCheck->setEnabled(!connected);
    m_statusLabel->setText(message + QLatin1Char(' '));
}

void PlotWindow::onTogglePanel(bool visible)
{
    m_togglePanelAction->setText(visible ? QStringLiteral("Hide panel")
                                         : QStringLiteral("Show panel"));
    AppPage *page = m_pages.value(m_stack->currentIndex());
    if (page && page->hasSidePanel())
        page->setSidePanelVisible(visible);
}

void PlotWindow::onPageChanged(int index)
{
    AppPage *page = m_pages.value(index);
    const bool hasPanel = page && page->hasSidePanel();
    m_togglePanelAction->setEnabled(hasPanel);
    if (hasPanel)
        page->setSidePanelVisible(m_togglePanelAction->isChecked());
}
