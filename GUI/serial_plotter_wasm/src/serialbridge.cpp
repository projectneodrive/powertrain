#include "serialbridge.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

#include <algorithm>
#include <cstring>

// ---------------------------------------------------------------------------
//  JS -> C++ transport
//
//  We deliberately do NOT use Module.ccall here: Qt's own link line sets
//  -sEXPORTED_RUNTIME_METHODS, which overrides anything we pass and leaves
//  ccall absent from the generated runtime (the build still succeeds, so the
//  failure would only show up as "no data ever arrives" in the browser).
//
//  Instead JavaScript copies raw bytes straight into these C++-owned buffers
//  through HEAPU8 and then calls the exported commit functions. That relies
//  only on EMSCRIPTEN_KEEPALIVE exports plus HEAPU8, both of which are always
//  present. It also lets the reader hand us undecoded bytes so UTF-8 decoding
//  happens once, in C++.
// ---------------------------------------------------------------------------

static constexpr int kRxBufSize = 16384;
static constexpr int kStatusBufSize = 512;
static char g_rxBuf[kRxBufSize];
static char g_statusBuf[kStatusBufSize];

extern "C" {

EMSCRIPTEN_KEEPALIVE char *serial_rx_buffer() { return g_rxBuf; }
EMSCRIPTEN_KEEPALIVE int serial_rx_capacity() { return kRxBufSize; }

EMSCRIPTEN_KEEPALIVE void serial_rx_commit(int len)
{
    if (len > 0 && len <= kRxBufSize)
        SerialBridge::instance().feedBytes(g_rxBuf, len);
}

EMSCRIPTEN_KEEPALIVE char *serial_status_buffer() { return g_statusBuf; }
EMSCRIPTEN_KEEPALIVE int serial_status_capacity() { return kStatusBufSize; }

EMSCRIPTEN_KEEPALIVE void serial_status_commit(int connected, int len)
{
    len = std::clamp(len, 0, kStatusBufSize);
    SerialBridge::instance().setStatus(connected != 0,
                                       QString::fromUtf8(g_statusBuf, len));
}

} // extern "C"

// ---------------------------------------------------------------------------
//  Web Serial API glue
// ---------------------------------------------------------------------------

EM_JS(void, js_serial_connect, (int baud), {
    // Copy a status string into the C++ status buffer and notify.
    function pushStatus(connected, msg) {
        const cap = Module["_serial_status_capacity"]();
        const bytes = new TextEncoder().encode(msg);
        const n = Math.min(bytes.length, cap);
        HEAPU8.set(bytes.subarray(0, n), Module["_serial_status_buffer"]());
        Module["_serial_status_commit"](connected, n);
    }

    (async function() {
        try {
            if (!navigator.serial) {
                pushStatus(0, 'Web Serial API unavailable — use Chrome/Edge over https or localhost');
                return;
            }
            const port = await navigator.serial.requestPort();
            await port.open({ baudRate: baud });

            Module.NEO = { port: port, keepReading: true };
            Module.NEO.writer = port.writable.getWriter();
            pushStatus(1, 'Connected @ ' + baud);

            const reader = port.readable.getReader();
            Module.NEO.reader = reader;
            try {
                while (Module.NEO && Module.NEO.keepReading) {
                    const res = await reader.read();
                    if (res.done)
                        break;
                    const chunk = res.value;
                    if (!chunk || !chunk.length)
                        continue;
                    // Hand the raw bytes over in buffer-sized slices; C++ does
                    // the UTF-8 decode and the line splitting.
                    const cap = Module["_serial_rx_capacity"]();
                    let off = 0;
                    while (off < chunk.length) {
                        const n = Math.min(cap, chunk.length - off);
                        HEAPU8.set(chunk.subarray(off, off + n),
                                   Module["_serial_rx_buffer"]());
                        Module["_serial_rx_commit"](n);
                        off += n;
                    }
                }
            } catch (e) {
                pushStatus(0, 'Read error: ' + e);
            } finally {
                try { reader.releaseLock(); } catch (e) {}
            }
        } catch (e) {
            pushStatus(0, 'Connect failed: ' + e);
        }
    })();
});

EM_JS(void, js_serial_disconnect, (), {
    function pushStatus(connected, msg) {
        const cap = Module["_serial_status_capacity"]();
        const bytes = new TextEncoder().encode(msg);
        const n = Math.min(bytes.length, cap);
        HEAPU8.set(bytes.subarray(0, n), Module["_serial_status_buffer"]());
        Module["_serial_status_commit"](connected, n);
    }

    (async function() {
        const N = Module.NEO;
        if (!N)
            return;
        N.keepReading = false;
        try { if (N.reader) await N.reader.cancel(); } catch (e) {}
        try { if (N.writer) N.writer.releaseLock(); } catch (e) {}
        try { if (N.port) await N.port.close(); } catch (e) {}
        Module.NEO = null;
        pushStatus(0, 'Disconnected');
    })();
});

EM_JS(void, js_serial_write, (const char *text), {
    const N = Module.NEO;
    if (!N || !N.writer)
        return;
    const data = new TextEncoder().encode(UTF8ToString(text));
    N.writer.write(data).catch(function(e) {});
});

#endif // __EMSCRIPTEN__

// ---------------------------------------------------------------------------
//  C++ side
// ---------------------------------------------------------------------------

SerialBridge &SerialBridge::instance()
{
    static SerialBridge s;
    return s;
}

void SerialBridge::connectPort(int baud)
{
#ifdef __EMSCRIPTEN__
    js_serial_connect(baud);
#else
    Q_UNUSED(baud);
    setStatus(false, QStringLiteral("Serial is only available in the WebAssembly build"));
#endif
}

void SerialBridge::disconnectPort()
{
#ifdef __EMSCRIPTEN__
    js_serial_disconnect();
#else
    setStatus(false, QStringLiteral("Disconnected"));
#endif
}

void SerialBridge::writeLine(const QString &text)
{
    QString line = text;
    while (line.endsWith(QLatin1Char('\n')) || line.endsWith(QLatin1Char('\r')))
        line.chop(1);
    line.append(QLatin1Char('\n'));
#ifdef __EMSCRIPTEN__
    const QByteArray utf8 = line.toUtf8();
    js_serial_write(utf8.constData());
#else
    Q_UNUSED(line);
#endif
}

void SerialBridge::feedBytes(const char *utf8, int len)
{
    m_rxBuffer.append(QString::fromUtf8(utf8, len));

    int nl;
    while ((nl = m_rxBuffer.indexOf(QLatin1Char('\n'))) >= 0) {
        QString line = m_rxBuffer.left(nl);
        m_rxBuffer.remove(0, nl + 1);
        if (line.endsWith(QLatin1Char('\r')))
            line.chop(1);
        if (!line.isEmpty())
            emit lineReceived(line);
    }

    // Guard against a peer that never sends a newline.
    if (m_rxBuffer.size() > 65536)
        m_rxBuffer.clear();
}

void SerialBridge::setStatus(bool connected, const QString &message)
{
    m_connected = connected;
    if (!connected)
        m_rxBuffer.clear();
    emit statusChanged(connected, message);
}
