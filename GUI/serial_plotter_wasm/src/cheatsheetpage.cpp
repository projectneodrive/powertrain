#include "cheatsheetpage.h"
#include "serialbridge.h"

#include <QAbstractItemView>
#include <QFont>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace {

struct Cmd {
    const char *command;
    const char *example;
    const char *description;
    bool sendable;          // argument-less -> offer a Send button
};

// Mirrors handleSerial() in src/main.cpp. Keep in sync when commands change.
const Cmd kCommands[] = {
    {"A", "A", "Arm — enter closed loop. Runs a one-time calibration on first arm (the motor twitches; keep it free).", true},
    {"I", "I", "Idle — disarm, back to the safe state.", true},
    {"C", "C", "Clear latched errors / e-stop.", true},
    {"M", "M", "Measure phase resistance & inductance (motor free, disarmed).", true},
    {"H", "H", "Hall-only: calibrate the hall sector angles (motor free, disarmed; spins ~10 s).", true},
    {"B<duty>", "B0.1", "Brake-resistor test pulse (disarmed). duty 0..0.25.", false},
    {"V<rad/s>", "V10", "Velocity setpoint — switches to velocity mode.", false},
    {"T<Nm>", "T0.5", "Torque setpoint — switches to torque mode.", false},
    {"X<rad>", "X3.14", "Position setpoint — switches to position mode.", false},
    {"KP<v>", "KP0.3", "Velocity PID proportional gain (Nm per rad/s).", false},
    {"KI<v>", "KI2", "Velocity PID integral gain.", false},
    {"KD<v>", "KD0", "Velocity PID derivative gain.", false},
    {"K", "K", "Re-apply and print the current velocity PID gains.", true},
    {"LC<A>", "LC5", "Current limit (A), clamped to CFG_CURRENT_LIMIT_MAX.", false},
    {"LV<rad/s>", "LV20", "Velocity limit (rad/s), clamped to CFG_VEL_LIMIT_MAX.", false},
    {"G<v>", "G1.0", "Position controller P gain.", false},
    {"Q", "Q", "Dump the live config as a 'cfg …' line (used by Motor Config).", true},
};

enum Column { ColCmd, ColExample, ColDesc, ColAction, ColCount };

} // namespace

CheatSheetPage::CheatSheetPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    auto *intro = new QLabel(QStringLiteral(
        "Serial commands accepted by the firmware. Type any of these in the "
        "Live Plotter command box, or use the Send buttons here for the "
        "argument-less ones. Values use rad, rad/s, Nm and A."));
    intro->setWordWrap(true);
    layout->addWidget(intro);

    const int rows = int(sizeof(kCommands) / sizeof(kCommands[0]));
    auto *table = new QTableWidget(rows, ColCount, this);
    table->setHorizontalHeaderLabels({QStringLiteral("Command"),
                                      QStringLiteral("Example"),
                                      QStringLiteral("Description"),
                                      QString()});
    table->verticalHeader()->setVisible(false);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setWordWrap(true);
    table->horizontalHeader()->setSectionResizeMode(ColCmd, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(ColExample, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(ColDesc, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(ColAction, QHeaderView::ResizeToContents);

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
        if (c.sendable) {
            auto *btn = new QPushButton(QStringLiteral("Send %1").arg(QString::fromUtf8(c.command)));
            const QString cmd = QString::fromUtf8(c.command);
            connect(btn, &QPushButton::clicked, this, [cmd] {
                auto &bridge = SerialBridge::instance();
                if (bridge.isConnected())
                    bridge.writeLine(cmd);
            });
            table->setCellWidget(r, ColAction, btn);
        }
    }
    table->resizeRowsToContents();
    layout->addWidget(table, 1);
}
