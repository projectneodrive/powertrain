# Web GUI — live plotter, PID tuner & config over USB

A browser-based console for the board that plots telemetry, tunes the velocity
PID live, and reads/writes motor parameters — all over the board's **USB**
port, no install. It talks to the same firmware as the CAN interface, using the
serial commands documented in [Getting Started §5](Getting_Started.md#5-first-run-usb-serial).

Source and build instructions: [`GUI/serial_plotter_wasm/`](../GUI/serial_plotter_wasm/README.md).
(There is also a Python desktop version, [`GUI/serial_plotter_fast.py`](../GUI/serial_plotter_fast.py),
if you prefer not to use a browser.)

## Opening it

- **Hosted:** open the GitHub Pages URL for this repo (`…github.io/<repo>/plotter/`)
  in **Chrome or Edge**.
- **Local:** from `GUI/serial_plotter_wasm/` run `./build.ps1 -Run`, which serves
  it at `http://localhost:8000`.

### Browser requirements (important)

The GUI reaches the USB port through the browser's **Web Serial API**, which has
constraints a normal web page doesn't:

- **Chromium-based browser only** — Chrome, Edge, Brave, Opera. **Firefox and
  Safari do not support Web Serial** and won't work.
- Must be served over **https** or **localhost** (GitHub Pages is https, so
  that's fine) — not opened as a `file://`.
- On **Linux**, a snap/flatpak Chromium is sandboxed away from serial devices;
  install Chrome from the `.deb`, and make sure your user is in the `dialout`
  group.

## Connecting

1. Plug the board's USB into the PC.
2. Click **Connect (USB)** in the top bar and pick the board's port in the
   browser prompt (the picker only appears on that click).
3. Telemetry starts streaming; the graphs **clear automatically** on each new
   connection.

No hardware handy? Tick **Demo** in the top bar to feed synthetic telemetry (and
a sample config) so every page is usable offline.

## The pages

Four buttons in the top bar switch what you see; the USB connection, telemetry
and log are shared across all of them.

- **Live Plotter** — the command console. Type any serial command (see the
  Commands page), set the time window, export the log to CSV.
- **PID Tuner** — live closed-loop control: arm/idle, clear errors, pick
  Velocity/Torque/Position mode, push a setpoint, and edit the **velocity,
  current and position** PID gains (KP/KI/KD each) while watching the response.
  **Read from board (Q)** loads the live configuration — it fills every gain box
  and shows a summary of the limits and all three PID loops. Live Plotter and
  PID Tuner **share the same graphs** — switching between them only swaps the
  left panel.
- **Motor Config** — a table of motor parameters. **Read from board (Q)** pulls
  the live values; the writable rows (current/velocity limit, position gain,
  velocity PID) can be edited and pushed back with **Apply changes**. Hardware
  constants (pole pairs, KV, phase R/L …) are read-only.
- **Commands** — a cheat sheet of every serial command the firmware accepts,
  with a **Send** button for the argument-less ones.

> **Firmware:** the Motor Config and PID pages use serial commands
> (`LC/LV/G/Q`, `KP/KI/KD`) that must be present in the flashed firmware. If
> the board was flashed from an older build, reflash it (`pio run -t upload`).

## Working with the graphs

- Stacked charts (Target / Iq / Vel / Pos / Vbus, plus the sensorless
  obsdV / blend on hall builds). They're **compact** and share the height so
  several are visible at once; the area **scrolls** when the window is too short.
- **Show/hide** individual graphs with the **Visible graphs** checkboxes on the
  Live Plotter panel — untick the ones you don't care about and the rest expand
  to fill the space.
- **Drag a chart's header** (the `⠿ Name` strip) up or down to **reorder** the
  charts.
- **Resize** a chart by dragging the divider between two charts — make the one
  you care about taller and shrink the rest.
- **Hide panel** (Ctrl+B) collapses the left control panel for full-width plots.
- **Clear graphs** buttons are on both the Plotter and Tuner panels.

## Safety

Same as any other control path: arming runs a one-time calibration that
**twitches the motor**, so keep it free on first arm and keep the PSU
current-limited. See the safety note in
[Getting Started §1](Getting_Started.md#a-word-on-safety).
