// Motor configuration table. "Read from board" sends Q and fills the table
// from the firmware's "cfg ..." reply; editable rows can be pushed back with
// the matching serial commands (LC/LV/G/KP/KI/KD). Hardware constants
// (pole pairs, KV, phase R/L, ...) are shown read-only.
#pragma once

#include <QHash>
#include <QString>
#include <QVector>
#include <QWidget>

QT_BEGIN_NAMESPACE
class QLabel;
class QTableWidget;
QT_END_NAMESPACE

class ConfigPage : public QWidget
{
    Q_OBJECT
public:
    explicit ConfigPage(QWidget *parent = nullptr);

private slots:
    void onReadClicked();
    void onApplyClicked();
    void onConfigReceived(const QHash<QString, double> &fields);

private:
    // One table row. cmdPrefix != nullptr means the value is writable over
    // serial with that command (e.g. "LC" -> "LC4.0").
    struct ParamDef {
        const char *key;         // key in the firmware's cfg dump
        const char *label;
        const char *unit;
        const char *cmdPrefix;   // serial command prefix, or nullptr = read-only
        int decimals;
    };

    void setStatus(const QString &text);

    QTableWidget *m_table = nullptr;
    QLabel *m_statusLabel = nullptr;
    QVector<ParamDef> m_params;
    QHash<QString, double> m_lastRead;   // to detect edited rows
};
