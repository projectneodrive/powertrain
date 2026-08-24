# Web GUI — live plotter, PID tuner & config over USB

A browser-based console for the board that plots telemetry, tunes the velocity
PID live, and reads/writes motor parameters — all over the board's **USB**
port, no install. It talks to the same firmware as the CAN interface, using the
serial commands documented in [Getting Started §5](Getting_Started.md#5-first-run-usb-serial).

Source and build instructions: [`GUI/serial_plotter_wasm/`](../GUI/serial_plotter_wasm/README.md).
(There is also a Python desktop version, [`GUI/serial_plotter_fast.py`](../GUI/serial_plotter_fast.py),
with the same features — graphs + checkboxes, PID tuner, config read-back — if
you prefer a native window over a browser. Run it with
`python GUI/serial_plotter_fast.py`; it needs `pip install pyserial PySide6 pyqtgraph numpy`.)

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

Five buttons in the top bar switch what you see; the USB connection, telemetry
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
  the live values; the writable rows (the limits and all three PID loops) can be
  edited and pushed back with **Apply changes**. Hardware constants (pole pairs,
  KV, phase R/L …) are read-only, because no serial command can write them.
  Every row — its label, unit, precision and write command — comes from
  [`include/config_schema.h`](../include/config_schema.h), the same table the
  firmware generates its `Q` reply from and the ESP32 station generates its own
  from. Adding a parameter there adds the row here.
- **CAN Devices** — the state of the CAN link, when you are connected to the
  **ESP32 control station** ([`can_utilities`](../can_utilities/README.md))
  rather than to the board directly. See below.
- **Commands** — a cheat sheet of every serial command the firmware accepts,
  with a **Send** button for the argument-less ones.

## The CAN Devices page

Fed by the `can …` status line the control station emits once a second. It
exists because this information is **state, not events**: node id, link, the
four error words and nine bus counters are only meaningful as a live table, and
printing them as prose is what used to make the serial monitor unreadable.

- **Header** — node id and link status at a glance, green/amber/red.
- **Device** — axis state and control mode *decoded into words*, heartbeat age,
  and the heartbeat period / link timeout the station derived from the
  firmware's own CAN table. **Safety stop** shows whether the link-loss stop is
  armed, disabled, or has *fired* — a disarmed axis and an axis nobody armed
  look identical otherwise. Once it has fired, press **Arm (A)** when the link
  is back.
- **Bus** — the ESP32 TWAI controller's state and every error counter. Counters
  climbing while the state stays `RUNNING` is the missing-terminator signature,
  so those cells tint amber rather than leaving you to spot them in a column of
  zeros.
- **Error words** — axis / motor / encoder / controller, in hex *and* decoded:
  `0x140  MOTOR_FAILED | ENCODER_FAILED`. The names come from
  [`include/axis_vocab.h`](../include/axis_vocab.h), the same table the firmware
  and the station compile, so a new error bit is named in all three at once.
  A bit this build has no name for is shown as `unnamed 0x…` rather than
  dropped — that is the one worth knowing about.
- **Nodes seen on the bus** — every node id observed, target marked. A node-id
  mismatch shows up here without turning any tracing on.
- **CAN functions** — arm / idle / clear errors, plus **E-stop** and **Reboot**,
  which are CANSimple frames with no equivalent on the board's own console, and
  **Refresh fault codes** (those words are answered on request, not broadcast).
  The page also states plainly which commands CANSimple cannot carry at all
  (`KD`, `JP/JI/JD`, `PI/PD`, `H`) — those need the board's USB port.
- **Station log level** — how much the ESP32 *sends*. `D3` adds a per-frame CAN
  trace; the default `D2` keeps it off.

Connected straight to the **board's** USB port instead, no `can …` lines arrive.
The page says so, and falls back to the frame counters the firmware publishes on
its telemetry line — the honest subset.

## Reading the serial monitor

The monitor pane under the graphs is filtered and colour-coded by severity.
**Monitor shows** (Live Plotter panel) picks the floor: *Errors only*,
*Warnings and above*, *Info and above* (default) or *Everything*.

Two filters, doing different jobs — worth keeping straight:

| Filter | Where | Effect |
|---|---|---|
| **Monitor shows** | Live Plotter panel | Hides lines *here*. They are still received, and still captured to CSV — a recording never omits what you had hidden. |
| **Station log level** (`D0`–`D3`) | CAN Devices page | Stops the ESP32 **sending** them at all. Use this one when the link itself is the bottleneck. |

The station's own log is edge-triggered and deduplicated, so a repeated fault
appears once followed by `previous line repeated N times` rather than as a wall
of identical lines. Anything the board emits that predates this format — its
boot banner, every `AK …` acknowledgement — is still shown, as Info.

> **Firmware:** the Motor Config and PID pages use serial commands
> (`LC/LV/G/Q`, `KP/KI/KD`) that must be present in the flashed firmware. If
> the board was flashed from an older build, reflash it (`pio run -t upload`).

## Working with the graphs

- Stacked charts (Target / Iq / Vel / Pos / Vbus / Regen / Brake, plus the
  sensorless **blend** and **obs dV** on hall builds — see below). They're **compact** and share the height so
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

## Commissioning the sensorless observer

Two of the charts exist for one job. On hall builds the firmware streams:

- **blend** — how much of the angle and velocity comes from the flux observer
  rather than the halls. 0 is pure hall, 1 is pure observer, and anything
  between is the handoff across
  `[CFG_SENSORLESS_VEL_LO, CFG_SENSORLESS_VEL_HI]`.
- **obs dV** — the observer's *disagreement* with the hall, `v_obs - v_hall`
  in rad/s. This is the number that decides whether the handoff is safe.

`motor_config.h` states the procedure and it is worth following: keep
`CFG_SENSORLESS_ENABLE` at 0, spin the motor over its whole range, and confirm
**obs dV stays near zero** before turning the handoff on. `HybridSensor` gates
the handoff on that same value (tolerance `0.5 + 0.15·|v_hall|`), so an observer
that disagrees makes **blend** chatter between 0 and 1 — and every flip is a step
change in the velocity the PID sees. On the plots that reads as erratic motion
in the crossover band, with the velocity trace jumping by rad/s at a time while
`Iq` shows no torque that could have caused it.

## Safety

Same as any other control path: arming runs a one-time calibration that
**twitches the motor**, so keep it free on first arm and keep the PSU
current-limited. See the safety note in
[Getting Started §1](Getting_Started.md#a-word-on-safety).

Driving through the ESP32 control station adds one behaviour worth knowing
about: if the heartbeat stops for a few seconds (`BRIDGE_LINK_LOSS_STOP_MS`,
3 s by default) it **disarms the axis once**, so the motor coasts rather than
running on with nobody watching it. It does not repeat — commands you type
afterwards still reach the bus, which is the point when the bus is the thing
that broke. The CAN Devices page's *Safety stop* row says whether it has fired.
Details in [`can_utilities/README.md`](../can_utilities/README.md#safety).
