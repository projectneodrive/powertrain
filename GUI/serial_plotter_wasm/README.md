# Serial plotter — Qt for WebAssembly

A C++/Qt port of [`serial_plotter_fast.py`](../serial_plotter_fast.py), compiled to
**WebAssembly** so it runs in a browser tab, while still talking to the
**ODrive v3.6** board over **USB**.

It parses the same `key=value` telemetry emitted by the firmware
(`t= mode= tgt= Iq= vel= pos= Vbus=` — see [`src/main.cpp`](../../src/main.cpp)
`SerialTask`), plots Target / Iq / Vel / Pos / Vbus on stacked live charts, shows
the raw serial monitor, sends the single-letter commands (`A`, `I`, `M`, `C`,
`T<val>`, `V<val>`, `KP/KI/KD<val>`), and can export the log to CSV.

## How USB works in the browser (important)

Qt's `QSerialPort` **does not exist** in a WebAssembly build — a sandboxed
browser tab has no OS serial access. Instead this app uses the browser's
**[Web Serial API](https://developer.mozilla.org/docs/Web/API/Web_Serial_API)**
(`navigator.serial`), bridged into C++ by a small `EM_JS` layer in
[`src/serialbridge.cpp`](src/serialbridge.cpp). Consequences:

- **Browser:** Chromium-based only — Chrome, Edge, Opera, Brave. Firefox and
  Safari do not implement Web Serial.
- **Secure context:** the page must be served over `https://` **or** from
  `http://localhost`. Opening the `.html` via `file://` will not work.
- **User gesture:** the port picker only appears on a click. That is why the
  **Connect** button is what opens the port.
- The ODrive v3.6 exposes a USB CDC (virtual COM) port; Web Serial's `baudRate`
  is required but effectively ignored by USB CDC, so the 115200 default is fine.

## Prerequisites

1. **Qt for WebAssembly** with the **Qt Charts** module, via the Qt online
   installer. In the component tree tick *WebAssembly (single-threaded)*,
   *Qt Charts* (under Additional Libraries) and a desktop kit (MinGW 64-bit).
2. The **matching Emscripten SDK**. Each Qt version is pinned to exactly one
   emsdk version — a mismatch is the most common build failure. Read the pin
   straight out of your own install rather than guessing:

   ```powershell
   Select-String -Path "C:\Qt\<ver>\wasm_singlethread\lib\cmake\Qt6\QtPublicWasmToolchainHelpers.cmake" `
                 -Pattern "QT_EMCC_RECOMMENDED_VERSION"
   ```

   **Qt 6.8.3 → emsdk 3.1.56** (verified on this project).

   ```powershell
   cd C:\; git clone https://github.com/emscripten-core/emsdk.git
   cd C:\emsdk; .\emsdk install 3.1.56; .\emsdk activate 3.1.56
   ```

## Build

Use the `qt-cmake` wrapper from the **wasm** kit — it points CMake at the
Emscripten toolchain automatically. Verified working on Windows with
Qt 6.8.3 + emsdk 3.1.56:

```powershell
. C:\emsdk\emsdk_env.ps1                       # see gotchas below
$env:PATH = "C:\Qt\Tools\Ninja;" + $env:PATH   # Qt bundles Ninja, off PATH
cd GUI\serial_plotter_wasm
& "C:\Qt\6.8.3\wasm_singlethread\bin\qt-cmake.bat" -S . -B build -G Ninja
cmake --build build
```

```bash
# Linux/macOS equivalent
source ~/emsdk/emsdk_env.sh
cd GUI/serial_plotter_wasm
~/Qt/6.8.3/wasm_singlethread/bin/qt-cmake -S . -B build -G Ninja
cmake --build build
```

### Windows/PowerShell gotchas

- **Use `&` before the quoted `qt-cmake.bat` path.** PowerShell parses a
  leading quoted string as a string literal, not a command
  (`Jeton inattendu « -S »`).
- **Use `emsdk_env.ps1`, not `emsdk_env.bat`.** The `.bat` sets variables in a
  child `cmd` that immediately exits, so PowerShell sees nothing. Dot-source
  the `.ps1`. Either way it only affects the *current* shell — re-run it in
  every new terminal.
- **Ninja** ships with Qt at `C:\Qt\Tools\Ninja` but isn't on PATH. Add it, or
  drop `-G Ninja`.
- If a configure fails, **delete `build/`** before retrying — a stale
  `CMakeCache.txt` pins the broken generator.
- Harmless noise: `Qt6Protobuf`/`Qt6ProtobufTools` "not found" warnings come
  from an optional QML/VirtualKeyboard plugin chain. This app uses neither.

The build produces, in `build/`:
`serial_plotter_wasm.html`, `.js`, `.wasm`, `qtloader.js`, plus font/`.data`
files.

## Run

Serve the `build/` directory over HTTP (not `file://`) and open the page in
Chrome/Edge:

```bash
cd build
python -m http.server 8000
# then open http://localhost:8000/serial_plotter_wasm.html
```

Click **Connect (USB)**, choose the ODrive's serial port in the browser prompt,
and the plots start streaming. Use the command box or the `A / I / M / C` quick
buttons to drive the firmware, and **Save CSV** to download the captured log.

## Notes / limitations

- **Don't reintroduce `Module.ccall` in the serial bridge.** Qt sets
  `-sEXPORTED_RUNTIME_METHODS` on its own link line and the last value wins, so
  anything we pass is silently dropped and `ccall` is absent from the generated
  runtime. The build still succeeds — the only symptom is that no serial data
  ever arrives. The bridge therefore calls `EMSCRIPTEN_KEEPALIVE` exports and
  moves bytes through `HEAPU8` instead.
- Single-threaded Qt WASM build. The firmware emits ~10 telemetry lines/s, so
  Qt Charts keeps up comfortably; the heavy ring-buffer/downsampling tricks from
  the Python version are unnecessary here.
- Reconnecting after unplugging the board may require re-picking the port
  (the browser drops the handle when the device disappears).
- The same source also compiles as a **desktop** Qt app (for UI work); on
  desktop the serial bridge is a stub since Web Serial is browser-only.
