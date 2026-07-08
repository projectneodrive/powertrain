# Odrive_Powertrain_Neodrive

Custom **SimpleFOC + FreeRTOS** firmware for an **ODrive v3.6 clone (MKS,
single‑channel, STM32F405 + DRV8301)**, plus the host‑side tooling to configure
and drive it. The firmware replaces the stock ODrive firmware but keeps the
**ODrive CANSimple** protocol, so existing ODrive CAN tools keep working. Target
application: e‑bike motor control (torque / velocity, with sensor or sensorless).

## 👉 Start here

**[docs/Getting_Started.md](docs/Getting_Started.md)** — a from‑zero guide:
install the toolchain, flash the board with an ST‑Link, and drive it over CAN
(with a full Arduino example and the complete command reference).

## Repository layout

The PlatformIO firmware project **is the repository root**.

| Path | What it is |
|------|-----------|
| [`platformio.ini`](platformio.ini), [`src/`](src/), [`include/`](include/), [`lib/`](lib/) | **The firmware.** `src/main.cpp` (FreeRTOS tasks + axis state machine), `include/board_config.h` (pins / limits / timing), `lib/odrive_can/` (CANSimple protocol layer). |
| [`test/`](test/) | Standalone bench sketches (raw encoder read, open‑loop, closed‑loop) — **not** part of the main build. |
| [`CAN/`](CAN/) | CAN tooling: the ODrive CANSimple **DBC generator** (`create_can_dbc.py`) and an **Arduino CAN sender** example (`arduino_can_sender/`). |
| [`docs/`](docs/) | Documentation (start with `Getting_Started.md`). |

## Firmware status

- ✅ Hardware‑timer encoder (fixes the FreeRTOS scheduler starvation from the old
  software‑interrupt encoder)
- ✅ FreeRTOS task architecture (20 kHz FOC loop, safety, CAN, telemetry)
- ✅ Safe‑state boot: disarmed until a CAN `Set_Axis_State(CLOSED_LOOP)` (or serial `A`)
- ✅ ODrive CANSimple interface — torque / velocity / position, switchable at
  runtime; heartbeat + telemetry
- ✅ Phase‑current sensing (DRV8301 + low‑side shunts) → `foc_current` torque control

### What SimpleFOC does today

| Feature | Status |
|---------|--------|
| Sensor **offset + direction autocalibration** (`initFOC`, runs on arm) | ✅ (not yet persisted → re‑runs each power‑up) |
| Velocity control (PID) / position control (P) / torque | ✅ |
| **Phase‑current sensing** → true Nm torque + current limiting (`foc_current`) | ✅ (falls back to voltage torque if current‑sense init fails) |
| **Motor R/L autocalibration** (`characteriseMotor`, CAN `MOTOR_CALIBRATION` / serial `M`) | ✅ |
| **Hall** sensor support | ❌ active — only the HW quadrature encoder is wired (Phase 5) |
| **Sensorless** (BEMF/flux observer) | ❌ (Phase 7; SimpleFOC has no built‑in observer) |
| Config / calibration **persistence to flash** | ❌ (Phase 5/6b) |

## Hardware quick facts

STM32F405RGT6 @ 168 MHz · DRV8301 6‑PWM (TIM1) · encoder/hall PB4/PB5(/PC9) ·
DRV SPI3 CS PC13 · phase current PC0/PC1 · Vbus PA6 · CAN1 PB8/PB9. Full pin map
in [`include/board_config.h`](include/board_config.h).
