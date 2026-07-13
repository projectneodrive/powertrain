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

## Repository layout

The PlatformIO firmware project **is the repository root**.

| Path | What it is |
|------|-----------|
| [`platformio.ini`](platformio.ini), [`src/`](src/), [`include/`](include/), [`lib/`](lib/) | **The firmware.** `src/main.cpp` (FreeRTOS tasks + axis state machine), `include/board_config.h` (pins / limits / timing), `lib/odrive_can/` (CANSimple protocol layer). |
| [`test/`](test/) | Standalone bench sketches (raw encoder read, open‑loop, closed‑loop) — **not** part of the main build. |
| [`CAN/`](CAN/) | CAN tooling: the ODrive CANSimple **DBC generator** (`create_can_dbc.py`) plus **Arduino MCP2515** and **ESP32 TWAI** sender examples (`arduino_can_sender/`, `esp32_twai_sender/`). |
| [`docs/`](docs/) | Documentation (start with `Getting_Started.md`). |

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
in [`include/board_config.h`](include/board_config.h).

For host-side CAN control, the repo includes both an Arduino MCP2515 sender and
an ESP32 native TWAI sender. The ESP32 sketch expects an external 3.3 V CAN
transceiver such as the CJMCU-230.

Two compile‑time switches there: **`MOTOR_PRESET`** (`BENCH` 7pp+encoder /
`EBIKE` 26pp+hall) and **`SENSOR_TYPE`** (`SENSOR_TYPE_QUADRATURE` /
`SENSOR_TYPE_HALL`). `SENSOR_TYPE` defaults from the preset but can be overridden
in `platformio.ini` (`-D SENSOR_TYPE=SENSOR_TYPE_HALL`).
