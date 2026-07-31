# Serial plotter — Qt for WebAssembly

A C++/Qt port of [`serial_plotter_fast.py`](../serial_plotter_fast.py), compiled to
**WebAssembly** so it runs in a browser tab, while still talking to the
**ODrive v3.6** board over **USB**.

It parses the same `key=value` telemetry emitted by the firmware
(`t= mode= tgt= Iq= vel= pos= Vbus=` — see [`src/main.cpp`](../../src/main.cpp)
`SerialTask`), plots Target / Iq / Vel / Pos / Vbus on stacked live charts, shows
the raw serial monitor, sends commands, and can export the log to CSV.

## Pages

Three toolbar buttons switch what you see; the USB connection, telemetry stream
and log are shared throughout (one connection, opened once).

| Button | What it does |
|--------|--------------|
| **Live Plotter** | Serial command console (`A/I/M/C`, `T`, `V`, `KP/KI/KD`, `Q`, …), time-window control, CSV export. |
| **PID Tuner** | Live closed-loop control: arm/idle, clear errors, pick Velocity/Torque/Position mode, push a setpoint, edit KP/KI/KD, and a Clear-graphs button. |
| **Motor Config** | A table of motor parameters. **Read from board (Q)** pulls the live values; writable rows (current/velocity limit, position gain, velocity PID) can be edited and pushed back with **Apply changes**. Hardware constants (pole pairs, KV, phase R/L, …) are read-only. |

**Live Plotter and PID Tuner share the same graphs and log** — switching between
them only swaps the left control panel, so the plots keep running underneath.
Motor Config replaces the whole view.

Graph behaviour:

- The five charts are **compact**: they share the height so all fit a normal
  window, and the area only scrolls once it gets too short.
- **Drag a chart's header** (the `⠿ Name` strip) up or down to reorder the
  charts.
- **Resize** a chart by dragging the divider between two charts (they sit in a
  vertical splitter).
- The graphs **clear automatically on each new connection**, and there's a
  Clear button on both the Plotter and Tuner panels.
- **Hide panel** (Ctrl+B) collapses the left panel for full-width plots.

### Firmware serial protocol used by the pages

The Config and PID pages rely on serial commands added to
[`src/main.cpp`](../../src/main.cpp) `handleSerial()`:

| Command | Meaning |
|---------|---------|
| `LC<A>` | set current limit (clamped to `CFG_CURRENT_LIMIT_MAX`) |
| `LV<rad/s>` | set velocity limit (clamped to `CFG_VEL_LIMIT_MAX`) |
| `G<val>` | set position P gain (alias of `PP`) |
| `X<rad>` | position setpoint (enters position mode) |
| `KP/KI/KD<val>` | velocity PID gains |
| `JP/JI/JD<val>` | current PID gains (V/A) |
| `PP/PI/PD<val>` | position PID gains |
| `Q` | dump the live config as one `cfg key=val …` line for the Config & PID Tuner pages |

**You must reflash the firmware** for the Config/PID pages to work against real
hardware; an older build simply won't answer `Q` or accept `LC/LV/G/X`. In
**Demo** mode the app fabricates a `cfg` line so both pages are usable with no
board attached.

### Adding a telemetry channel (one place)

The plotted channels are defined once, in
[`include/telemetry_schema.h`](../../include/telemetry_schema.h), and shared by
both sides of the wire: the firmware's `SerialTask` generates its `key=value`
output from it, and the GUI's [`src/channels.h`](src/channels.h) generates the
chart/checkbox list from the *same* file. To add a signal, add one
`TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr)` line there — it then
streams from the board and shows up as a new graph (with its own **Visible
graphs** checkbox) automatically. `expr` is a firmware-side value expression the
GUI never compiles; use `TELEMETRY_CHANNEL_HALL` for signals that only exist on
hall builds. (Demo values are synthetic, so give a new channel one in
`src/demosource.cpp` too if you want it to animate offline.)

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

## Build — the short way

[`build.ps1`](build.ps1) wraps all the environment setup below:

```powershell
.\build.ps1                             # build WASM
.\build.ps1 -Run                        # build WASM, serve on :8000, open browser
.\build.ps1 -Deploy                     # build + copy to <repo>/docs/plotter (GitHub Pages)
.\build.ps1 -Target desktop -Run        # build + launch the native app
.\build.ps1 -Target desktop -Run -Demo  # ...with synthetic telemetry, no hardware
.\build.ps1 -Clean                      # wipe the build dir first
```

Override paths with `-QtVersion 6.8.3 -QtRoot C:\Qt -EmsdkRoot C:\emsdk`.

## Publishing to GitHub Pages

The build emits **`index.html`** (a copy of the Qt-generated
`serial_plotter_wasm.html`) plus `.nojekyll`, so the output folder is directly
publishable. Don't rename the loader by hand — the next build regenerates
`serial_plotter_wasm.html` and your renamed copy silently goes stale.

```powershell
.\build.ps1 -Deploy          # builds, then copies to <repo>/docs/plotter
```

Then commit `docs/plotter/` and, in the repository settings, set
**Pages → Deploy from a branch → main → /docs**. The app lands at:

```
https://<user>.github.io/<repo>/plotter/
```

It deploys to `docs/plotter/`, not `docs/`, on purpose: `docs/` already holds
the project's markdown documentation, and a `.nojekyll` at the docs root would
stop GitHub rendering those `.md` files. Override with
`-DeployDir <path>` if you prefer somewhere else.

**Web Serial works on Pages** — it requires a secure context and GitHub serves
HTTPS. The single-threaded Qt build also avoids needing `COOP`/`COEP` headers,
which GitHub Pages cannot set (the *multithreaded* Qt WASM kit would not work
there for exactly that reason).

Two things to weigh before committing:

- The payload is **~14 MB**, dominated by `serial_plotter_wasm.wasm`, but
  GitHub Pages serves it gzipped so the actual transfer is **~5 MB**. Fine for
  Pages (limits: 1 GB per site, 100 MB per file), though it does land in git
  history on every rebuild — consider a `gh-pages` branch or a CI build if you
  redeploy often.
- First load downloads the whole `.wasm`; after that the browser caches it.

## Size / performance

The WASM is built **MinSizeRel** with a link-time **`-Oz`** and the smaller
**emmalloc** allocator (see [`CMakeLists.txt`](CMakeLists.txt)). This matters a
lot: a default configure leaves `CMAKE_BUILD_TYPE` empty, which compiles at
`-O0` — that produced a 21.6 MB, sluggish binary. The current settings give:

| | wasm (raw) | over the wire (gzip) |
|---|---|---|
| `-O0` (unoptimised default) | 21.6 MB | ~8 MB |
| MinSizeRel + `-Oz` | **~14 MB** | **~5 MB** |

The optimisation flags (not dropping features) are what did the work — the
weight is the Qt runtime, not the app code.

## Developing without hardware (demo mode)

Tick **Demo data (no hardware)** in the Connection box, or pass `--demo` on the
desktop build. It synthesises firmware-style telemetry at the same 10 Hz the
real `SerialTask` uses — a stepped velocity target with a first-order lag on
`vel`, integrated `pos`, an `Iq` that tracks the error, and a rippling `Vbus`,
plus the occasional `AK V:` log line to exercise the monitor pane.

The generated bytes go through `SerialBridge::feedBytes()` — the *same* path
real serial data takes — so demo mode also exercises the UTF-8 decode, the line
splitting and the `key=value` parser, not just the charts.

Demo mode works in the browser build too, which is handy for checking layout
without plugging in the ODrive.

### Iteration speed

Measured on this project (incremental rebuild, one `.cpp` touched):

| target | full | incremental |
|---|---|---|
| WASM | ~90 s | ~20 s |
| desktop | ~39 s | ~17 s |

Compile times are close, so desktop's real advantage is the **run** loop: no
web server, no browser reload, and a usable debugger. Use desktop + `--demo`
for UI/parser/plot work; use the WASM build to verify real USB behaviour.

## IntelliSense (VS Code)

Qt headers are resolved from the CMake-generated `compile_commands.json`, so
run a desktop build once first:

```powershell
.\build.ps1 -Target desktop
```

Then open **`neodrive.code-workspace`** at the repo root
(*File > Open Workspace from File...*). It is a multi-root workspace so the
firmware and the Qt GUI each keep their own C/C++ configuration — the repo root
keeps PlatformIO's auto-generated `c_cpp_properties.json`, and this folder has
its own in [`.vscode/c_cpp_properties.json`](.vscode/c_cpp_properties.json).
Neither overwrites the other.

Pick the active config via the C/C++ configuration selector in the status bar:
*Qt Desktop (MinGW)* or *Qt WebAssembly (Emscripten)*.

> Do **not** add Qt paths to the repo-root `.vscode/c_cpp_properties.json` —
> PlatformIO regenerates that file and your edits will vanish.

## Build — manually

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
  the QPainter plots keep up trivially; the heavy ring-buffer/downsampling
  tricks from the Python version are unnecessary here.
- Reconnecting after unplugging the board may require re-picking the port
  (the browser drops the handle when the device disappears).
- The same source also compiles as a **desktop** Qt app (for UI work); on
  desktop the serial bridge is a stub since Web Serial is browser-only.
