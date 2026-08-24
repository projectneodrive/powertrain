#include "configpage.h"
#include "serialbridge.h"
#include "telemetryhub.h"

#include <QAbstractItemView>
#include <QBrush>
#include <QColor>
#include <QEvent>
#include <QHBoxLayout>
#include <QPalette>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <cmath>

namespace {
enum Column { ColParam = 0, ColValue = 1, ColUnit = 2, ColCount = 3 };
}

ConfigPage::ConfigPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    auto *intro = new QLabel(QStringLiteral(
        "Read the live configuration from the board, edit the writable rows "
        "(white background), then Apply. Greyed rows are compile-time hardware "
        "constants and cannot be changed over serial."));
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto *buttonRow = new QHBoxLayout;
    auto *readButton = new QPushButton(QStringLiteral("Read from board (Q)"));
    auto *applyButton = new QPushButton(QStringLiteral("Apply changes"));
    buttonRow->addWidget(readButton);
    buttonRow->addWidget(applyButton);
    buttonRow->addStretch(1);
    layout->addLayout(buttonRow);

    m_table = new QTableWidget(kNumConfigParams, ColCount, this);
    m_table->setHorizontalHeaderLabels({QStringLiteral("Parameter"),
                                        QStringLiteral("Value"),
                                        QStringLiteral("Unit")});
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setSectionResizeMode(ColParam, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(ColValue, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(ColUnit, QHeaderView::ResizeToContents);
    m_table->setSelectionMode(QAbstractItemView::NoSelection);

    for (int r = 0; r < kNumConfigParams; ++r) {
        const ParamDef &p = kConfigParams[r];
        auto *nameItem = new QTableWidgetItem(QString::fromUtf8(p.label));
        nameItem->setFlags(Qt::ItemIsEnabled);
        m_table->setItem(r, ColParam, nameItem);

        auto *valueItem = new QTableWidgetItem(QStringLiteral("—"));
        valueItem->setFlags(p.cmdPrefix ? (Qt::ItemIsEnabled | Qt::ItemIsEditable)
                                        : Qt::ItemIsEnabled);
        m_table->setItem(r, ColValue, valueItem);

        auto *unitItem = new QTableWidgetItem(QString::fromUtf8(p.unit));
        unitItem->setFlags(Qt::ItemIsEnabled);
        m_table->setItem(r, ColUnit, unitItem);
    }
    applyReadOnlyTint();
    layout->addWidget(m_table, 1);

    m_statusLabel = new QLabel(QStringLiteral("Not read yet."));
    m_statusLabel->setWordWrap(true);
    layout->addWidget(m_statusLabel);

    connect(readButton, &QPushButton::clicked, this, &ConfigPage::onReadClicked);
    connect(applyButton, &QPushButton::clicked, this, &ConfigPage::onApplyClicked);
    connect(&TelemetryHub::instance(), &TelemetryHub::configReceived,
            this, &ConfigPage::onConfigReceived);
}

void ConfigPage::onReadClicked()
{
    auto &bridge = SerialBridge::instance();
    if (!bridge.isConnected()) {
        setStatus(QStringLiteral("Connect to the USB serial port first."));
        return;
    }
    bridge.writeLine(QStringLiteral("Q"));
    setStatus(QStringLiteral("Requested config (Q) — waiting for reply…"));
}

void ConfigPage::onConfigReceived(const QHash<QString, double> &fields)
{
    m_lastRead = fields;
    int filled = 0;
    for (int r = 0; r < kNumConfigParams; ++r) {
        const ParamDef &p = kConfigParams[r];
        const QString key = QString::fromLatin1(p.key);
        auto *item = m_table->item(r, ColValue);
        if (fields.contains(key)) {
            item->setText(QString::number(fields.value(key), 'f', p.decimals));
            ++filled;
        }
    }
    setStatus(QStringLiteral("Config read — %1 of %2 fields populated.")
                  .arg(filled).arg(kNumConfigParams));
}

void ConfigPage::onApplyClicked()
{
    auto &bridge = SerialBridge::instance();
    if (!bridge.isConnected()) {
        setStatus(QStringLiteral("Connect to the USB serial port first."));
        return;
    }

    int sent = 0;
    for (int r = 0; r < kNumConfigParams; ++r) {
        const ParamDef &p = kConfigParams[r];
        if (!p.cmdPrefix)
            continue;

        bool ok = false;
        const double value = m_table->item(r, ColValue)->text().toDouble(&ok);
        if (!ok)
            continue;

        // Only push rows that actually changed from what the board reported,
        // so re-applying doesn't spam unchanged setpoints.
        const QString key = QString::fromLatin1(p.key);
        if (m_lastRead.contains(key) && std::abs(value - m_lastRead.value(key)) < 1e-9)
            continue;

        bridge.writeLine(QString::fromLatin1(p.cmdPrefix)
                             + QString::number(value, 'f', p.decimals));
        ++sent;
    }

    if (sent == 0) {
        setStatus(QStringLiteral("No changes to apply."));
        return;
    }
    // Read back so the table reflects what the firmware actually accepted
    // (values are clamped to safety ceilings on the board).
    bridge.writeLine(QStringLiteral("Q"));
    setStatus(QStringLiteral("Applied %1 change(s); re-reading…").arg(sent));
}

void ConfigPage::setStatus(const QString &text)
{
    m_statusLabel->setText(text);
}

void ConfigPage::applyReadOnlyTint()
{
    // A fixed near-transparent black is invisible in dark mode. Derive the tint
    // from the current palette so read-only rows stand out in both themes.
    const QColor base = palette().color(QPalette::Base);
    const bool dark = base.lightness() < 128;
    const QColor tint = dark ? QColor(255, 255, 255, 34) : QColor(0, 0, 0, 24);
    const QBrush brush(tint);

    for (int r = 0; r < kNumConfigParams; ++r) {
        if (kConfigParams[r].cmdPrefix)     // writable row -> no tint
            continue;
        for (int c = 0; c < m_table->columnCount(); ++c)
            if (QTableWidgetItem *it = m_table->item(r, c))
                it->setBackground(brush);
    }
}

void ConfigPage::changeEvent(QEvent *event)
{
    QWidget::changeEvent(event);
    if (event->type() == QEvent::PaletteChange && m_table)
        applyReadOnlyTint();
}
