// Motor configuration table. "Read from board" sends Q and fills the table from
// the firmware's "cfg ..." reply; editable rows can be pushed back with the
// serial command the schema names for them. Hardware constants (pole pairs, KV,
// phase R/L, ...) have no such command and are shown read-only.
//
// The rows themselves come from configparams.h, i.e. from the schema the
// firmware compiles -- this class holds no parameter list of its own.
#pragma once

#include <QHash>
#include <QString>
#include <QWidget>

#include "configparams.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QTableWidget;
QT_END_NAMESPACE

class ConfigPage : public QWidget
{
    Q_OBJECT
public:
    explicit ConfigPage(QWidget *parent = nullptr);

protected:
    void changeEvent(QEvent *event) override;   // re-tint on light/dark switch

private slots:
    void onReadClicked();
    void onApplyClicked();
    void onConfigReceived(const QHash<QString, double> &fields);

private:
    void setStatus(const QString &text);
    void applyReadOnlyTint();

    QTableWidget *m_table = nullptr;
    QLabel *m_statusLabel = nullptr;
    QHash<QString, double> m_lastRead;   // to detect edited rows
};
