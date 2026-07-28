// Read-only reference of every serial command the firmware accepts, mirroring
// handleSerial() in src/main.cpp. Argument-less commands have a Send button so
// the page doubles as a quick control surface.
#pragma once

#include <QWidget>

class CheatSheetPage : public QWidget
{
    Q_OBJECT
public:
    explicit CheatSheetPage(QWidget *parent = nullptr);
};
