# Odrive_Powertrain_Neodrive

Custom **SimpleFOC + FreeRTOS** firmware for an **ODrive v3.6 clone (MKS,
single‑channel, STM32F405 + DRV8301)**, plus the host‑side tooling to configure
and drive it. The firmware replaces the stock ODrive firmware but keeps the
**ODrive CANSimple** protocol, so existing ODrive CAN tools keep working. Target
application: e‑bike motor control (torque / velocity, with sensor or sensorless).

## 👉 Start here

- **[docs/Getting_Started.md](docs/Getting_Started.md)** — from‑zero guide:
  install the toolchain, flash the board with an ST‑Link, and drive it over CAN
  (with a full Arduino example and the complete command reference).
- **[docs/Calibration.md](docs/Calibration.md)** — commissioning a **new motor**:
  which parameters to find, how to measure `R`/`L` + sensor offset/direction, and
  how to save them so the board boots pre‑calibrated.
- **[src/README.md](src/README.md)** — **working on the firmware itself**: how it
  is organised (an IEC 61131‑3 style PLC), and how to add a program, a task, a
  serial/CAN command or a telemetry channel. Continues into
  **[docs/Architecture.md](docs/Architecture.md)** for the runtime internals,
  concurrency model, timing budget and how to change core components.
- **[docs/GUI.md](docs/GUI.md)** — the browser‑based **live plotter / PID tuner /
  motor config / CAN devices** web GUI (talks to the board over USB, no install). Hosted at
  `https://projectneodrive.github.io/powertrain/plotter/` (Chrome/Edge only —
  [Web Serial API](https://developer.mozilla.org/docs/Web/API/Web_Serial_API)),
  or run it locally from [`GUI/serial_plotter_wasm/`](GUI/serial_plotter_wasm/README.md).

## Repository layout

The PlatformIO firmware project **is the repository root**.

| Path | What it is |
|------|-----------|
| [`platformio.ini`](platformio.ini), [`src/`](src/), [`include/`](include/), [`lib/`](lib/) | **The firmware**, organised as an IEC 61131‑3 style PLC — tasks, programs, function blocks and an explicit process image. **Start with [`src/README.md`](src/README.md)**, then `src/config/configuration.cpp` (the task table + boot order). Configuration lives in `include/config/` (`hw_pinout.h` / `motor_config.h` / `plc_config.h`); `lib/odrive_can/` is the CANSimple fieldbus driver. |
| [`GUI/`](GUI/) | Host‑side USB tooling: the Qt‑for‑WebAssembly **web GUI** (`serial_plotter_wasm/`, see [docs/GUI.md](docs/GUI.md)) and a Python desktop plotter (`serial_plotter_fast.py`). Both talk directly to the board's USB serial console. |
| [`test/`](test/) | Standalone bench sketches (raw encoder read, open‑loop, closed‑loop) — **not** part of the main build. |
| [`CAN/`](CAN/) | CAN tooling: the ODrive CANSimple **DBC generator** (`create_can_dbc.py`) plus bare‑minimum **Arduino MCP2515** and **ESP32 TWAI** one‑way sender *examples* (`arduino_can_sender/`, `esp32_twai_sender/`) — send‑only, no telemetry parsing. For an actual CAN control station, use `can_utilities` instead (see below). |
| [`can_utilities/`](can_utilities/README.md) | **ESP32 CANSimple control station** — a separate PlatformIO project, built from its own directory. Compiles the firmware's tables in `include/` so the two ends of the CAN bus cannot drift. See below. |
| [`docs/`](docs/) | Documentation (start with `Getting_Started.md`). |

### CAN host tooling: `can_utilities`

The real host‑side CAN control/telemetry tool is
**[`can_utilities/`](can_utilities/README.md)** — a **separate PlatformIO
project inside this repository** (build it from that directory, not from the
root). It turns an ESP32 (native TWAI, no external CAN library needed beyond a
3.3 V transceiver) into a full CANSimple bridge:

- **Control:** arm / idle / e‑stop / clear‑errors / reboot, torque / velocity /
  position setpoints, and a spring‑centered potentiometer wired up as a live
  velocity joystick (with runtime rest‑point calibration).
- **Motor info:** encoder position/velocity, Iq setpoint/measured, bus V/I, the
  heartbeat's axis state + error, and on‑demand per‑subsystem motor/encoder/
  controller error codes.
- **Configuration:** velocity/current limits, position P gain, velocity PID
  P+I gains (CANSimple's `Set_Vel_Gains` has no D term — that stays UART‑only
  on the board's own serial console).
- **Connection diagnostics:** per‑frame TX/RX logging, bus error/bus‑off
  alerts, node‑ID mismatch detection, and periodic link‑health counters —
  built from debugging this exact link (see the project's own history for
  what a marginal CAN link actually looks like from both sides).

It exposes the same `key=value` telemetry line and the same serial commands, so
`GUI/serial_plotter_fast.py` or the web GUI above can be pointed at either the
board's direct USB port **or** this ESP32's USB‑CDC port interchangeably.

That interchangeability is enforced, not maintained by hand: `can_utilities`
compiles the firmware's **own** tables — `include/can_ids.h`,
`can_commands.h`, `console_commands.h`, `telemetry_schema.h` and
`config/motor_config.h` — so the node id, bit rate, limits, command set and
telemetry channels have exactly one definition. Adding a command or a channel on
the firmware side **fails the bridge's build** until it says how that maps onto
CANSimple, or that it doesn't. See
[`can_utilities/README.md`](can_utilities/README.md).

## Firmware status

- ✅ Hardware‑timer encoder (fixes the FreeRTOS scheduler starvation from the old
  software‑interrupt encoder)
- ✅ FreeRTOS task architecture (20 kHz FOC loop, safety, CAN, telemetry)
- ✅ Safe‑state boot: disarmed until a CAN `Set_Axis_State(CLOSED_LOOP)` (or serial `A`)
- ✅ ODrive CANSimple interface — torque / velocity / position, switchable at
  runtime; heartbeat + telemetry
- ✅ Phase‑current sensing (DRV8301 + low‑side shunts) → `foc_current` torque control
  with real current limiting (Vbus is read without disturbing the shunt ADC)
- ✅ Hall angle interpolation (`SmoothingSensor`) — smooth commutation despite the
  60°‑electrical hall resolution
- ✅ **Live velocity‑PID tuning**: CAN `Set_Vel_Gains` (0x01B) or serial `KP`/`KI`/`KD`

### What SimpleFOC does today

| Feature | Status |
|---------|--------|
| Sensor **offset + direction autocalibration** (`initFOC`, runs on arm) | ✅ (not yet persisted → re‑runs each power‑up) |
| Velocity control (PID) / position control (P) / torque | ✅ |
| **Phase‑current sensing** → true Nm torque + current limiting (`foc_current`) | ✅ (falls back to voltage torque if current‑sense init fails) |
| **Motor R/L autocalibration** (`characteriseMotor`, CAN `MOTOR_CALIBRATION` / serial `M`) | ✅ |
| **Hall** sensor support | ✅ compile‑time select (`SENSOR_TYPE` = quadrature ↔ hall) |
| **Sensorless** (BEMF/flux observer) | ❌ (Phase 7; SimpleFOC has no built‑in observer) |
| Config / calibration **persistence to flash** | ❌ (Phase 5/6b) |

## Hardware quick facts

STM32F405RGT6 @ 168 MHz · DRV8301 6‑PWM (TIM1) · encoder/hall PB4/PB5(/PC9) ·
DRV SPI3 CS PC13 · phase current PC0/PC1 · Vbus PA6 · CAN1 PB8/PB9. Full pin map
in [`include/config/hw_pinout.h`](include/config/hw_pinout.h).

For host-side CAN control, use [`can_utilities/`](can_utilities/README.md) (an
ESP32 + 3.3 V CAN transceiver such as the CJMCU-230) — see above.
`CAN/arduino_can_sender/` and `CAN/esp32_twai_sender/` remain in this repo as
minimal, send-only reference examples.

Motor/sensor parameters (`CFG_POLE_PAIRS`, `CFG_KV`, etc.) are plain `#define`s
there. One compile‑time switch remains: **`SENSOR_TYPE`** (`SENSOR_TYPE_QUADRATURE` /
`SENSOR_TYPE_HALL`, defaults to hall), which can be overridden in `platformio.ini`
(`-D SENSOR_TYPE=SENSOR_TYPE_QUADRATURE`).
