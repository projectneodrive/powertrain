// Common base for the app's top-level pages (Live Plotter, Motor Config, PID
// Tuner). The shell (PlotWindow) uses this to query a page's title and to drive
// the shared "Hide panel" toggle without knowing the concrete type.
//
// Intentionally has no Q_OBJECT: it declares no signals/slots of its own, and
// keeping it moc-free avoids needing a matching .cpp just for the meta-object.
// The concrete pages carry Q_OBJECT themselves.
#pragma once

#include <QString>
#include <QWidget>

class AppPage : public QWidget
{
public:
    using QWidget::QWidget;

    virtual QString pageTitle() const = 0;
    virtual bool hasSidePanel() const { return false; }
    virtual void setSidePanelVisible(bool) {}
};
