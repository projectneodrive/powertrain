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

| Path | What it is |
|------|-----------|
| [`Neodrive_test/`](Neodrive_test/) | **The firmware** (PlatformIO). `src/main.cpp`, `include/board_config.h`, `lib/odrive_can/` (CANSimple layer). |
| [`CAN/`](CAN/) | CAN tooling: the ODrive CANSimple **DBC generator** (`create_can_dbc.py`) and an **Arduino CAN sender** example (`arduino_can_sender/`). |
| [`docs/`](docs/) | Documentation (start with `Getting_Started.md`). |
| [`Documentation/`](Documentation/) | Notes/recipes for the **stock ODrive** firmware (hall & sensorless config). |
| [`Odrive_configuration_power_supply_V1.py`](Odrive_configuration_power_supply_V1.py) | Interactive host script to configure a stock ODrive (hall, torque mode). |

## Firmware status

- ✅ Hardware‑timer encoder (fixes the FreeRTOS scheduler starvation from the old
  software‑interrupt encoder)
- ✅ FreeRTOS task architecture (20 kHz FOC loop, safety, CAN, telemetry)
- ✅ ODrive CANSimple interface — torque / velocity / position, switchable at
  runtime; heartbeat + telemetry
- ⏳ Phase‑current sensing (DRV8301 + low‑side shunts) for true Nm torque
- ⏳ Runtime‑selectable hall / quadrature / sensorless sensing
- ⏳ Config persistence to flash

## Hardware quick facts

STM32F405RGT6 @ 168 MHz · DRV8301 6‑PWM (TIM1) · encoder/hall PB4/PB5(/PC9) ·
DRV SPI3 CS PC13 · phase current PC0/PC1 · Vbus PA6 · CAN1 PB8/PB9. Full pin map
in [`Neodrive_test/include/board_config.h`](Neodrive_test/include/board_config.h).
