# Getting Started — Flashing the MKS/ODrive board & controlling it over CAN

This guide is written for someone who has **never** used an STM32, PlatformIO, or CAN
bus before. It walks you all the way from "I have a board and an ST‑Link" to
"I'm spinning the motor with CAN commands from an Arduino."

**What the board is:** an ODrive v3.6 clone (MKS single‑channel) with an
STM32F405 micro‑controller and a DRV8301 gate driver. Instead of the stock ODrive
firmware it runs our custom **SimpleFOC + FreeRTOS** firmware (this repository),
which speaks the **ODrive CANSimple** protocol.

---

## Table of contents
1. [What you need](#1-what-you-need)
2. [Install the software](#2-install-the-software-to-compile)
3. [Wire the ST‑Link](#3-wire-the-st-link)
4. [Build & flash the firmware](#4-build--flash-the-firmware)
5. [First run (USB serial)](#5-first-run-usb-serial)
6. [The CAN command scheme](#6-the-can-command-scheme)
7. [Wire the CAN bus](#7-wire-the-can-bus)
8. [Arduino example: send CAN commands](#8-arduino-example-send-can-commands)
9. [Full command reference](#9-full-command-reference)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. What you need

### Hardware
| Item | Notes |
|------|-------|
| MKS / ODrive v3.6 board | STM32F405, DRV8301, single channel (M0) |
| **ST‑Link V2** programmer | The cheap blue/silver USB clones work fine |
| BLDC motor | + a quadrature encoder **or** hall sensors |
| Power supply | ~24 V bench PSU (current‑limited is safest for first tests) |
| USB cable | Micro‑USB to the board, for the serial monitor |
| For CAN: | a USB‑CAN adapter **or** an Arduino Uno/Nano/ESP32 + MCP2515 module |
| 2× 120 Ω resistors | CAN bus termination |

![Board + ST-Link overview](images/board_overview.png)
> 📷 **Image to add — `images/board_overview.png`:** a photo of the MKS/ODrive
> board with the ST‑Link, motor and PSU connected, labelled. (You can shoot this
> on the bench.)

### A word on safety
The firmware boots in a **safe, disarmed state** — the driver is off and the
motor is free; it does **nothing** until you arm it. Arming (over CAN, or the
serial `A` command) runs a one‑time **calibration** that briefly energises and
**twitches the motor**, so keep it free/mounted when you first arm, and keep the
PSU current‑limited (1–2 A) for first tests.

---

## 2. Install the software (to compile)

We use **PlatformIO**, which lives inside **VS Code** and downloads the STM32
compiler and libraries for you automatically.

1. **Install VS Code** — <https://code.visualstudio.com/>
2. **Install the PlatformIO IDE extension**: open VS Code → Extensions (the
   squares icon, or `Ctrl+Shift+X`) → search **"PlatformIO IDE"** → Install.
   Let it finish its one‑time setup (it installs a Python core in the background).
3. **Windows only — ST‑Link USB driver:** install ST's
   **STSW‑LINK009** driver so Windows recognises the ST‑Link
   (<https://www.st.com/en/development-tools/stsw-link009.html>).
   On Linux/macOS no driver is needed (you may need a udev rule on Linux).

![PlatformIO extension in VS Code](images/platformio_install.png)
> 📷 **Image to add — `images/platformio_install.png`:** screenshot of the
> "PlatformIO IDE" extension page in VS Code's Extensions panel.

4. **Open the project:** in VS Code → *File → Open Folder…* and select the
   **repository root** folder (the one containing `platformio.ini`). PlatformIO
   detects it and, on the first build, downloads:
   - the `ststm32` platform + Arduino framework + ARM GCC toolchain,
   - the libraries in `platformio.ini` (SimpleFOC, SimpleFOCDrivers, STM32
     FreeRTOS, STM32_CAN).

   The first build therefore takes a few minutes and needs internet. After that
   it's fast and offline.

---

## 3. Wire the ST‑Link

The ST‑Link talks to the STM32 over **SWD** (Serial Wire Debug). You only need
**3 wires** (4 if the board isn't otherwise powered):

| ST‑Link pin | Board SWD pin |
|-------------|---------------|
| GND         | GND           |
| SWDIO       | SWDIO (PA13)  |
| SWCLK       | SWCLK (PA14)  |
| 3.3 V *(optional)* | 3.3 V — **only if the board is NOT powered from its own PSU** |

On the ODrive v3.6 the SWD pins are on the small debug header near the STM32.

⚠️ **Do not** connect the ST‑Link 3.3 V if the board is already powered by its
24 V PSU — power it one way only. Connecting `NRST` too (if available) enables
"connect under reset", which helps if flashing is unreliable.

![ST-Link to SWD wiring](images/stlink_wiring.png)
> 📷 **Image to add — `images/stlink_wiring.png`:** the 3–4 wire ST‑Link→SWD
> hookup. Good reference to redraw/photograph:
> [STM32‑base "Connecting your debugger"](https://stm32-base.org/guides/connecting-your-debugger.html).

---

## 4. Build & flash the firmware

With the project (repository root) open in VS Code, use the PlatformIO toolbar at
the bottom of the window:

- **Build** (the ✓ checkmark) — compiles. First time is slow (downloads).
- **Upload** (the → arrow) — compiles **and** flashes over the ST‑Link.

Or from a terminal in the repository root:
```bash
pio run              # build only
pio run -t upload    # build + flash via ST-Link
```

![PlatformIO build/upload toolbar](images/platformio_toolbar.png)
> 📷 **Image to add — `images/platformio_toolbar.png`:** the blue PlatformIO
> status bar showing the ✓ (build) and → (upload) icons.

A successful upload ends with something like `Programming Complete!` /
`Verified OK`.

---

## 5. First run (USB serial)

1. Plug the board's **USB** into your PC (separate from the ST‑Link).
2. Open the **Serial Monitor**: PlatformIO toolbar 🔌 icon, or
   `pio device monitor -b 115200`.
3. On boot you should see it come up **disarmed / safe** (no motion):
   ```
   --- SimpleFOC + FreeRTOS + CANSimple ---
   CAN up: node 0 @ 100000 bps
   SAFE state (disarmed). Arm to calibrate + run:
     CAN: Set_Axis_State(8)   or   serial: A
   t=... #0 mode=1 tgt=0.00 vel=0.00 Vbus=... SAFE
   ```
   The status word at the end of each line is `SAFE` (never armed) → `RUN`
   (armed & running) → `idle` (calibrated but disarmed) → `[FAULT]`.

4. **Arm it to make it move.** Arming runs a one‑time calibration (the motor
   twitches, so keep it free), then enters closed loop:
   - `A` → arm (equivalent to CAN `Set_Axis_State(8)`)
   - `V10` → velocity mode, 10 rad/s
   - `T0.5` → torque mode, 0.5 Nm (real current if current‑sense is active;
     q‑axis volts in the voltage fallback)
   - `M` → measure phase resistance/inductance (motor must be free)
   - `I` → disarm (back to safe) &nbsp; `C` → clear a latched fault

> **New motor?** Do the one‑time commissioning in **[Calibration.md](Calibration.md)**
> — measure `R`/`L` and the sensor offset/direction, save them into
> `board_config.h`, and the board boots pre‑calibrated (arms without the
> alignment sweep, so the rotor needn't be free).

> The steady `#N` counter proves the real‑time scheduler is healthy — it keeps a
> fixed 10 Hz cadence even while the motor spins fast (this is the bug we fixed
> by moving the encoder onto a hardware timer).

---

## 6. The CAN command scheme

The firmware speaks **ODrive CANSimple**. Three rules cover everything:

**1) The arbitration ID encodes *who* and *what*:**
```
  11-bit standard ID  =  (node_id << 5) | command_id
```
`node_id` is the board's address (default **0**, set by `CFG_CAN_NODE_ID`).
`command_id` is one of the numbers in the [reference table](#9-full-command-reference).
So for node 0, "Set Input Vel" (0x0D) is sent on ID `0x00D`; for node 3 it would
be `(3<<5)|0x0D = 0x06D`.

**2) Payloads are little‑endian.** Floats are IEEE‑754 32‑bit (4 bytes),
integers are `int32`/`uint32` (4 bytes). A frame carrying two values (e.g.
velocity + torque feed‑forward) puts the first at bytes 0–3 and the second at
4–7.

**3) Units are ODrive units:** **rev** and **rev/s** for position/velocity
(not radians), **Nm** for torque, **A** for current. The firmware converts to
its internal radians automatically.

A typical control session is just four messages:
```
Set_Axis_State(8)              -> arm (CLOSED_LOOP)
Set_Controller_Mode(2, 1)      -> velocity mode, passthrough
Set_Limits(10, 15)             -> 10 rev/s, 15 A ceiling
Set_Input_Vel(2.0)             -> go 2 rev/s   (repeat to change speed)
```
Meanwhile the board continuously sends back **Heartbeat** (0x01, state+errors)
and **Encoder_Estimates** (0x09, position+velocity).

---

## 7. Wire the CAN bus

CAN is a 2‑wire differential bus: **CANH** and **CANL**, plus a shared **GND**.

- Connect board **CANH ↔ CANH**, **CANL ↔ CANL**, and tie the grounds together.
- Put a **120 Ω resistor between CANH and CANL at each end** of the bus (so two
  total). Most MCP2515 blue boards already have one fitted.
- On the STM32 the CAN peripheral is on **PB8 (RX) / PB9 (TX)** feeding an
  on‑board transceiver.

⚠️ **Clone caveat:** verify your MKS clone actually populates the CAN
transceiver on PB8/PB9 — some don't. If `candump` shows nothing, check this
first (a scope/multimeter on CANH/CANL should show activity while the board
sends its heartbeat).

![CAN bus wiring](images/can_wiring.png)
> 📷 **Image to add — `images/can_wiring.png`:** MCP2515 ↔ board CANH/CANL/GND
> with the two 120 Ω terminators. Reference:
> [MCP2515 CAN network tutorial](https://lastminuteengineers.com/mcp2515-can-module-arduino-tutorial/).

---

## 8. Arduino example: send CAN commands

The full, commented sketch is at
[`CAN/arduino_can_sender/arduino_can_sender.ino`](../CAN/arduino_can_sender/arduino_can_sender.ino).
It uses an **Arduino Uno/Nano + MCP2515** module and the **`mcp_can`** library by
*coryjfowler* (install it from the Arduino IDE Library Manager).

**MCP2515 → Arduino wiring:** `SCK→D13  MISO→D12  MOSI→D11  CS→D10  INT→D2
VCC→5V  GND→GND`.

The heart of the sketch is these two ideas — build the ID, and pack
little‑endian floats:

```cpp
// arbitration id = (node_id << 5) | command_id
static uint32_t canId(uint8_t cmd) { return ((uint32_t)NODE_ID << 5) | cmd; }
static void putF32(uint8_t* b, float f) { memcpy(b, &f, 4); }  // little-endian

void setInputVel(float rev_s, float torque_ff = 0.0f) {
  uint8_t d[8];
  putF32(d,     rev_s);       // bytes 0..3 : velocity (rev/s)
  putF32(d + 4, torque_ff);   // bytes 4..7 : torque feed-forward (Nm)
  CAN.sendMsgBuf(canId(0x00D), 0 /*standard id*/, 8, d);
}
```

`setup()` arms the board and puts it in velocity mode; `loop()` reverses the
target every 3 seconds and prints the heartbeat / encoder telemetry it receives.
It demonstrates **every** command category: state, mode, velocity, torque,
position, limits, estop, clear‑errors, and reading telemetry.

**No Arduino? Use SocketCAN** (Linux, with a USB‑CAN adapter):
```bash
candump can0                    # watch: 0x001 heartbeat, 0x009 estimates
cansend can0 00B#0200000001000000   # Set_Controller_Mode: velocity(2), passthrough(1)
cansend can0 00D#0000004000000000   # Set_Input_Vel: 2.0 rev/s (float 2.0 = 0x40000000 LE)
```
Or drive it from Python with the generated DBC
([`CAN/create_can_dbc.py`](../CAN/create_can_dbc.py)) + `python-can` + `cantools`.

---

## 9. Full command reference

Node 0 shown (`ID = command_id`). For another node, add `node_id << 5`.
**→** = you send to the board, **←** = board sends to you.

### Commands you send (→ board)

| ID | Command | Payload (little‑endian) | Meaning |
|----|---------|-------------------------|---------|
| `0x002` | Estop | — | Emergency stop, latched |
| `0x007` | Set_Axis_State | `int32 state` | `1`=idle (disarm), `8`=closed‑loop (arm), `4`=motor cal (measure R/L), `5`=sensorless |
| `0x00B` | Set_Controller_Mode | `int32 mode, int32 input_mode` | mode `1`=torque `2`=velocity `3`=position; input `1`=passthrough |
| `0x00C` | Set_Input_Pos | `float pos(rev)` [`,int16 vel_ff, int16 torq_ff`] | position target |
| `0x00D` | Set_Input_Vel | `float vel(rev/s), float torque_ff(Nm)` | velocity target |
| `0x00E` | Set_Input_Torque | `float torque(Nm)` | torque target |
| `0x00F` | Set_Limits | `float vel_limit(rev/s), float current_limit(A)` | safety ceilings |
| `0x018` | Clear_Errors | — | clear latched errors / estop |
| `0x016` | Reboot | — | reset the MCU |
| `0x006` | Set_Axis_Node_ID | `int32 node_id` | change this board's CAN address |
| `0x01A` | Set_Pos_Gain | `float pos_gain` | position P gain |
| `0x01B` | Set_Vel_Gains | `float vel_gain, float vel_int_gain` | velocity PI gains |

### Telemetry the board sends (← board)

| ID | Message | Payload | Rate |
|----|---------|---------|------|
| `0x001` | Heartbeat | `uint32 axis_error, uint8 state, flags…` | 10 Hz |
| `0x009` | Encoder_Estimates | `float pos(rev), float vel(rev/s)` | 100 Hz |
| `0x014` | Get_Iq | `float iq_setpoint(A), float iq_measured(A)` | on request / cyclic |
| `0x017` | Get_Bus_Voltage_Current | `float voltage(V), float current(A)` | on request / cyclic |
| `0x003/4/5/1D` | Get_*_Error | `uint32 error` | on request |

> **Torque note:** `Set_Input_Torque` (Nm) is converted to a q‑axis current
> (`Iq = Nm / Kt`) and regulated by the FOC current loop, with `Get_Iq`
> reporting the real measured current. If the current‑sense hardware fails to
> initialise, the firmware falls back to applying the value as a q‑axis voltage.

This table matches the DBC in [`CAN/create_can_dbc.py`](../CAN/create_can_dbc.py),
so odrivetool and existing ODrive CAN tools work unchanged.

---

## 10. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| Upload fails / "no ST‑Link" | ST‑Link driver (Windows: STSW‑LINK009); check SWDIO/SWCLK/GND; try connecting `NRST`; power the board. |
| First build errors on a library | Needs internet on first build; re‑run. Paste the exact error if it persists. |
| Motor doesn't move | Is it armed? (`Set_Axis_State(8)` or serial). Is `Vbus` present? Check `nFAULT` — a `[FAULT]` on serial means the DRV8301 tripped (over‑current); reduce load and send `C`. |
| `candump` shows nothing | Bit rate mismatch (must be 100 kbit/s), missing 120 Ω terminators, swapped CANH/CANL, or the clone's CAN transceiver isn't populated (PB8/PB9). |
| MCP2515 won't init | Wrong crystal setting — try `MCP_16MHZ` vs `MCP_8MHZ` in the sketch. |
| `Vbus` reads wrong | Calibrate `CFG_VBUS_DIV` in `board_config.h` to your board's divider. |
| Motor spins the wrong way / jitters | Encoder A/B direction; the `STM32HWEncoder` is new — verify sign vs. the old software encoder. |

---

### See also
- **[Calibration.md](Calibration.md)** — commissioning a new motor (find & save
  `R`/`L`, sensor offset/direction, pre‑calibrated boot).
- Firmware source: [`src/main.cpp`](../src/main.cpp),
  [`include/board_config.h`](../include/board_config.h),
  [`lib/odrive_can/`](../lib/odrive_can/).
- ODrive CANSimple reference:
  [ODrive CAN guide](https://docs.odriverobotics.com/v/latest/guides/can-guide.html).
