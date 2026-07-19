// Bridge between the Qt/C++ world and the browser's Web Serial API.
//
// Why this exists: Qt's QSerialPort is not available in a WebAssembly build --
// a browser tab cannot open an OS serial device. The only portable way to talk
// to the ODrive v3.6's USB CDC port from inside a WASM page is the Web Serial
// API (navigator.serial), which lives in JavaScript. This class wraps that API
// with a small EM_JS glue layer (see serialbridge.cpp) and re-exposes it to the
// rest of the app as ordinary Qt signals/slots.
//
// On a normal desktop build (no Emscripten) the methods become no-ops that
// report "WASM only", so the UI still compiles and runs for development.
#pragma once

#include <QObject>
#include <QString>

class SerialBridge : public QObject
{
    Q_OBJECT
public:
    static SerialBridge &instance();

    bool isConnected() const { return m_connected; }

    // Ask the browser to prompt the user for a serial port, open it at `baud`
    // and start streaming. Must be triggered from a user gesture (button click)
    // or the browser will reject the port request.
    void connectPort(int baud);
    void disconnectPort();
    void writeLine(const QString &text);

    // --- called from JavaScript through the C shims in serialbridge.cpp.
    //     Do not call these directly from application code. ---
    void feedBytes(const char *utf8, int len);
    void setStatus(bool connected, const QString &message);

signals:
    void lineReceived(const QString &line);          // one complete '\n' line
    void statusChanged(bool connected, const QString &message);

private:
    SerialBridge() = default;

    QString m_rxBuffer;      // accumulates partial lines between reads
    bool m_connected = false;
};
