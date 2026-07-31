#include "cheatsheetpage.h"

#include <QAbstractItemView>
#include <QFont>
#include <QHeaderView>
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace {

struct Cmd {
    const char *command;
    const char *example;
    const char *description;
};

// Mirrors handleSerial() in src/main.cpp. Keep in sync when commands change.
const Cmd kCommands[] = {
    {"A", "A", "Arm — enter closed loop. Runs a one-time calibration on first arm (the motor twitches; keep it free)."},
    {"I", "I", "Idle — disarm, back to the safe state."},
    {"C", "C", "Clear latched errors / e-stop."},
    {"M", "M", "Measure phase resistance & inductance (motor free, disarmed)."},
    {"H", "H", "Hall-only: calibrate the hall sector angles (motor free, disarmed; spins ~10 s)."},
    {"V<rad/s>", "V10", "Velocity setpoint — switches to velocity mode."},
    {"T<Nm>", "T0.5", "Torque setpoint — switches to torque mode."},
    {"X<rad>", "X3.14", "Position setpoint — switches to position mode."},
    {"KP<v>", "KP0.3", "Velocity PID proportional gain (Nm per rad/s)."},
    {"KI<v>", "KI2", "Velocity PID integral gain."},
    {"KD<v>", "KD0", "Velocity PID derivative gain."},
    {"K", "K", "Re-apply and print the current velocity PID gains."},
    {"JP<v>", "JP1.0", "Current PID proportional gain (V per A)."},
    {"JI<v>", "JI50", "Current PID integral gain."},
    {"JD<v>", "JD0", "Current PID derivative gain."},
    {"PP<v>", "PP1.0", "Position PID proportional gain (same as G)."},
    {"PI<v>", "PI0", "Position PID integral gain."},
    {"PD<v>", "PD0", "Position PID derivative gain."},
    {"LC<A>", "LC5", "Current limit (A), clamped to CFG_CURRENT_LIMIT_MAX."},
    {"LV<rad/s>", "LV20", "Velocity limit (rad/s), clamped to CFG_VEL_LIMIT_MAX."},
    {"G<v>", "G1.0", "Position controller P gain (alias of PP)."},
    {"Q", "Q", "Dump the live config as a 'cfg …' line (used by Motor Config & PID Tuner)."},
};

enum Column { ColCmd, ColExample, ColDesc, ColCount };

} // namespace

// Page de RÉFÉRENCE uniquement : aucun bouton, aucune connexion au port série.
// Les commandes se tapent dans la boîte "Serial Commands" du Live Plotter.
// Sans widget dans les cellules, la table reste un simple modèle -> pas de
// hiérarchie de QPushButton à poser/redessiner à chaque changement de page.
CheatSheetPage::CheatSheetPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    auto *intro = new QLabel(QStringLiteral(
        "Serial commands accepted by the firmware — reference only. Type any of "
        "these in the Live Plotter command box. Values use rad, rad/s, Nm and A."));
    intro->setWordWrap(true);
    layout->addWidget(intro);

    const int rows = int(sizeof(kCommands) / sizeof(kCommands[0]));
    auto *table = new QTableWidget(rows, ColCount, this);
    table->setHorizontalHeaderLabels({QStringLiteral("Command"),
                                      QStringLiteral("Example"),
                                      QStringLiteral("Description")});
    table->verticalHeader()->setVisible(false);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setWordWrap(true);
    table->horizontalHeader()->setSectionResizeMode(ColCmd, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(ColExample, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(ColDesc, QHeaderView::Stretch);

    auto mono = [](QTableWidgetItem *it) {
        QFont f = it->font();
        f.setFamily(QStringLiteral("monospace"));
        f.setBold(true);
        it->setFont(f);
        return it;
    };

    for (int r = 0; r < rows; ++r) {
        const Cmd &c = kCommands[r];
        // fromUtf8 (not fromLatin1): the descriptions contain em-dashes, and
        // the source literals are UTF-8 -- fromLatin1 mangles multi-byte chars.
        table->setItem(r, ColCmd, mono(new QTableWidgetItem(QString::fromUtf8(c.command))));
        table->setItem(r, ColExample, mono(new QTableWidgetItem(QString::fromUtf8(c.example))));
        table->setItem(r, ColDesc, new QTableWidgetItem(QString::fromUtf8(c.description)));
    }
    table->resizeRowsToContents();
    layout->addWidget(table, 1);
}
