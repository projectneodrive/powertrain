# `can_utilities` — ESP32 CANSimple control station

A host-side control and telemetry station for the Neodrive powertrain board.
It turns an ESP32 (native TWAI — no CAN library needed beyond a 3.3 V
transceiver) into a full **ODrive CANSimple** master, and re-exports the board's
serial console and telemetry stream over USB.

The practical consequence: **[`GUI/serial_plotter_wasm`](../GUI/serial_plotter_wasm/)
and [`GUI/serial_plotter_fast.py`](../GUI/serial_plotter_fast.py) can be pointed
at this ESP32's USB port or at the board's own, interchangeably.** Same
commands, same `key=value` telemetry line, same `cfg ...` config line.

> This is a **separate PlatformIO project** living inside the firmware
> repository. Build it from this directory, not from the repo root. The two
> projects are compiled independently and share nothing but headers — see below,
> because that sharing is the whole design.
---

## Contents

- [The one idea](#the-one-idea)
- [Quick start](#quick-start)
- [Directory map](#directory-map)
- [Command reference](#command-reference)
- [Safety](#safety)
- [What CAN cannot reach](#what-can-cannot-reach)
- [The telemetry line](#the-telemetry-line)
- [Diagnostics](#diagnostics)
  - [Why the link keeps dropping](#why-the-link-keeps-dropping)
- [How to add something](#how-to-add-something)
- [Things that look arbitrary but are not](#things-that-look-arbitrary-but-are-not)

---

## The one idea

**Nothing about the protocol, the command set or the telemetry is defined in
this project.** It is all compiled from the firmware's own tables, one directory
up, through a single line in [`platformio.ini`](platformio.ini):

```ini
-I$PROJECT_DIR/../include
```

| What | Comes from | Used here for |
|---|---|---|
| CANSimple arbitration ids | [`../include/can_ids.h`](../include/can_ids.h) | `odcan::Cmd`, `arbId`/`nodeOf`/`cmdOf` |
| Which commands the board implements, which frames it broadcasts, how often | [`../include/can_commands.h`](../include/can_commands.h) | send-side checks, RX decoders, the link-loss timeout |
| The serial command set | [`../include/console_commands.h`](../include/console_commands.h) | this station's console + its help banner |
| The telemetry channels | [`../include/telemetry_schema.h`](../include/telemetry_schema.h) | the `key=value` line the GUI plots |
| Node id, bit rate, limits, gain defaults | [`../include/config/motor_config.h`](../include/config/motor_config.h) | `CFG_CAN_NODE_ID`, `CFG_CAN_BAUD`, `CFG_*_LIMIT*`, `CFG_VEL_P`… |
| Axis states, control modes, **error-bit names** | [`../include/axis_vocab.h`](../include/axis_vocab.h) | decoding a heartbeat into `IDLE -> CLOSED_LOOP` and an error word into `[MOTOR_FAILED\|ENCODER_FAILED]` instead of raw hex |

Only what is physically wired to **this ESP32** is defined locally, in
[`include/bridge_config.h`](include/bridge_config.h): transceiver pins,
potentiometer, serial rates.

### What breaks the build, and why that is the feature

This project is downstream of the firmware, so the tables get to fail it. All
four of these are verified to fire:

| Change on the firmware side | What happens here |
|---|---|
| New line in `console_commands.h` | **Compile error** — `cmdYourThing` not declared. The bridge must say how the command maps onto CANSimple, or that it does not. |
| New `TELEMETRY_CHANNEL` in `telemetry_schema.h` | **Link error** — `bridge::channel::yourkey` undefined. Somebody must decide whether that value is reachable over CAN. |
| New `CAN_TX_CYCLIC` in `can_commands.h` | **Link error** — `Axis::rx_sendYourThing` undefined. A new broadcast cannot be silently dropped. |
| Sending a command with no `CAN_RX` handler | **`static_assert`** at the send site: *"the board would ignore the frame"*. |
| `CFG_CAN_BAUD` changed to an unsupported rate | **`#error`** — no ESP32 TWAI timing constant for it. |

Before this existed, the two ends had already drifted, in ways that took bench
time to find rather than compiler time:

- `P` meant a **position setpoint** here and the **position-PID family** on the
  board. A GUI sending `P1.5` to the two ports did two unrelated things.
- Arming pushed a **15 A** current limit at a firmware configured for **4 A**.
- The bit rate was a hard-coded `500000` with a comment asking you to keep it in
  step with `CFG_CAN_BAUD`.
- The command-name table used for logging was hand-maintained, so a frame the
  board had learned to answer still logged as `UNKNOWN`.

---

## Quick start

### Hardware

| ESP32 pin | Goes to | Set in |
|---|---|---|
| `GPIO5` | transceiver **TXD** | `BRIDGE_TWAI_TX_PIN` |
| `GPIO4` | transceiver **RXD** | `BRIDGE_TWAI_RX_PIN` |
| `GPIO34` | potentiometer wiper (optional) | `POT_PIN` |
| 3V3 / GND | transceiver supply | — |

Use a **3.3 V** CAN transceiver (CJMCU-230 / SN65HVD230 or similar); a 5 V
MCP2551 will not do. `CANH`/`CANL` go to the board's `CAN1` (PB8/PB9 through its
own transceiver). **120 Ω termination at both ends of the bus** — a missing
terminator is the single most common cause of the error-counter climb described
under [Diagnostics](#diagnostics).

### Build and flash

```bash
cd can_utilities
pio run -t upload
pio device monitor        # 115200 baud
```

Then either drive it by hand from the monitor, or close it and point the web GUI
at the same port.

To drive a board configured for a different node id without editing either
config:

```bash
pio run -t upload --project-option="build_flags=-D BRIDGE_TARGET_NODE_ID=3"
```

If the firmware is built for a quadrature encoder, build this the same way
(`-D SENSOR_TYPE=SENSOR_TYPE_QUADRATURE`) so the two agree on whether the
hall-calibration command exists.

---

## Directory map

`src/main.cpp` is 8 lines: `begin()` and `poll()`. Everything else is a library.

```
      Serial (host GUI)                             CAN bus
            |                                          |
            v                                          v
      bridge_console  --commands-->  bridge_axis  <--frames-->  cansimple
            ^                             |                        |
            |                             v                     can_diag
      bridge_telemetry  <--readings--  bridge_state              (trace,
            |                             ^                       alerts,
            v                             |                      counters)
      Serial (host GUI)              pot_input
```

| Library | What it is | Rule it obeys |
|---|---|---|
| [`lib/cansimple/`](lib/cansimple/) | CANSimple over the ESP32 TWAI controller: framing, payload packing, TX/RX, and the compile-time queries generated from `can_commands.h`. | Knows what a *frame* is. Never knows what an *axis* is. |
| [`lib/can_bridge/`](lib/can_bridge/) | The application: the axis, the console, the telemetry line, the shared state. | The only place that knows both sides. |
| [`lib/can_diag/`](lib/can_diag/) | Frame trace, node discovery, bus alerts, health counters. | Pure observation — nothing here changes what is transmitted. |
| [`lib/logging/`](lib/logging/log.h) | Levelled, edge-triggered, deduplicated, rate-capped event log, plus the axis-name decoders. | Everything printed for a human goes through it. Nothing periodic does. |
| [`lib/pot_input/`](lib/pot_input/) | The spring-return potentiometer, read as a velocity joystick. | Knows nothing about CAN; returns a number, the caller decides. |

Inside `lib/can_bridge/`:

| File | Contents |
|---|---|
| [`bridge_state.h`](lib/can_bridge/bridge_state.h) | What the board told us (`Measured`) vs what we told the board (`Commanded`) — kept apart on purpose. |
| [`bridge_axis.{h,cpp}`](lib/can_bridge/bridge_axis.h) | One method per operator verb, one decoder per broadcast frame. **The only file that knows a payload's wire encoding**, including every rad ↔ rev conversion. |
| [`bridge_console.{h,cpp}`](lib/can_bridge/bridge_console.h) | The dispatch table: the firmware's commands, then this station's. |
| [`bridge_commands.h`](lib/can_bridge/bridge_commands.h) | X-macro list of the station's **extra** commands. |
| [`bridge_telemetry.{h,cpp}`](lib/can_bridge/bridge_telemetry.h) | The two machine-readable lines: one accessor per schema channel for `t=…`, and the `can …` status line. |
| [`can_bridge.{h,cpp}`](lib/can_bridge/can_bridge.h) | Assembly and the scan order. Read `poll()` first. |

The scan order in `poll()` mirrors the board's own COMMS task — **drain CAN, run
the logic, publish** — so a command issued in a given pass acts on that pass's
readings.

---

## Command reference

Everything the board's own console accepts, plus the station's extras. Matching
is case-insensitive; an exact two-letter match wins, otherwise the single-letter
entry catches the line.

### From the firmware's table

| Command | Does | Over CAN |
|---|---|---|
| `A` | arm closed loop | clear errors → velocity mode → 0 rad/s → `Set_Axis_State(CLOSED_LOOP)` |
| `I` | idle | `Set_Axis_State(IDLE)` |
| `V<rad/s>` | velocity setpoint | `Set_Controller_Mode` + `Set_Input_Vel` |
| `T<Nm>` | torque setpoint | `Set_Controller_Mode` + `Set_Input_Torque` |
| `X<rad>` | position setpoint | `Set_Controller_Mode` + `Set_Input_Pos` |
| `M` | measure phase R/L | `Set_Axis_State(MOTOR_CALIBRATION)` |
| `C` | clear errors | `Clear_Errors` |
| `KP<v>` `KI<v>` `K` | velocity PID P / I / reapply | `Set_Vel_Gains(P, I)` |
| `PP<v>` `P` | position P / reapply | `Set_Pos_Gain(P)` |
| `G<v>` | position gain | `Set_Pos_Gain(P)` |
| `LC<A>` | current limit | `Set_Limits`, clamped to `CFG_CURRENT_LIMIT_MAX` |
| `LV<rad/s>` | velocity limit | `Set_Limits`, clamped to `CFG_VEL_LIMIT_MAX` |
| `Q` | dump config as `cfg ...` | *(local — see below)* |

`Set_Limits` carries velocity **and** current in one frame, and `Set_Vel_Gains`
carries P **and** I. That is why `bridge_state.h` caches both halves: you cannot
change one without resending the other.

### The station's own (`lib/can_bridge/bridge_commands.h`)

| Command | Does |
|---|---|
| `E` | e-stop (`ESTOP` — a CAN frame with no console equivalent on the board) |
| `R` | reboot the board (`Reboot`) |
| `F` | print motor / encoder / controller error codes (axis word decoded), and request a refresh |
| `D<0-3>` | event log level — see [Diagnostics](#diagnostics). Bare `D` reports the current one |
| `Z` | capture the potentiometer's rest point — hold it at rest, then send `Z` |
| `?` | help banner |

A key here must not shadow one in the firmware's table. The two tables are built
independently so the compiler cannot catch it; `bridge_console.cpp` checks at
boot and complains on the console instead.

---

## Safety

### The link-loss stop

If the heartbeat stops for **`BRIDGE_LINK_LOSS_STOP_MS`** (3 s by default), the
station **disarms the axis** — `Set_Axis_State(IDLE)`, so the motor coasts
rather than actively braking. That is the same thing the firmware's own CAN
watchdog does (`CFG_WATCHDOG_MS`), from the other end of the bus.

Three details that are the design, not incidentals:

- **It is much longer than the link-lost timeout** (500 ms). Losing the link is
  worth *reporting* immediately; a dropped frame, a nudged connector or a board
  reboot is not worth stopping a motor for.
- **It fires once, on the edge — and never repeats.** Repeating it every scan
  would overwrite whatever you type next, and when the bus is the broken thing,
  the console is the only thing you have left. After the stop, commands keep
  going out normally.
- **It needs a falling edge.** With no board ever seen, nothing is stopped —
  there is no link to lose, and nothing was armed.

**It never counts time this station was not watching.** A scan takes
microseconds; anything over `BRIDGE_SCAN_STALL_MS` means the loop was blocked
(a serial write waiting on a host that stopped reading is the usual cause), and
that time is credited back to the heartbeat clock and reported as
`log W SYS loop stalled …`. Without it, a 3 s stall made the heartbeat age jump
past the link-lost *and* safety-stop thresholds in a single scan, so the station
declared a loss and disarmed the motor in the same breath — for a board that had
never stopped transmitting. Disarming a motor because *we* stopped looking is
the worst failure this thing can have.

The credit is **capped at one `BRIDGE_LINK_LOSS_STOP_MS` per outage**, which is
the other half of that argument: uncapped, a station stalling repeatedly would
push the reference forward forever and never disarm at all, leaving a motor
running with a genuinely dead master. Capped, the stop is at worst late by that
much — the right way round to be wrong.

A heartbeat coming back re-arms the one-shot, so a link that drops twice stops
the motor twice. Press `A` to re-arm the axis once it is back. Set
`BRIDGE_LINK_LOSS_STOP_MS` to `0` to disable it; a `static_assert` rejects a
value shorter than the link-lost timeout, which would disarm the axis before
the link was even declared down.

The `can …` line carries `stop_after=` and `stopped=`, so the GUI's CAN Devices
page shows whether the stop is armed, disabled, or has fired — a disarmed axis
and an axis nobody armed otherwise look identical.

### Potentiometer calibration at startup

The pot's rest point is measured at boot, in
[`Joystick::begin()`](lib/pot_input/pot_input.h). A spring-return pot is *at
rest* when the board powers up, which is the one moment it can be measured
without asking anybody to hold it there.

It replaces a default derived from an ohm-meter reading (160–4.3 kΩ travel,
3.3 kΩ at rest), which assumed the measured resistance span maps linearly onto
the full ADC swing. It does not — and being wrong there means the joystick
commands a velocity while the pot sits physically at rest.

`POT_CAL_SAMPLES` readings are averaged, and the **spread** decides whether the
average means anything:

| Rejected because | What it usually is |
|---|---|
| spread > `POT_CAL_MAX_SPREAD` | a hand still on the pot at power-up — or **nothing connected to the pin**, since a floating ESP32 ADC wanders much further than this |
| within `POT_CAL_RAIL_MARGIN` of either rail | full deflection at boot, or a broken wiper reading a supply rail |

It retries `POT_CAL_ATTEMPTS` times, then falls back to the estimate and warns,
naming which check failed. The joystick still works — its zero is just a guess
until you hold the pot at rest and send `Z`, which re-runs the same measurement.

The first `poll()` after a calibration never commands anything: the reference
the pot is measured against just moved, and without re-seeding it the next
sample reads as a large operator movement.

## What CAN cannot reach

CANSimple has no representation for these, so they answer with
*"not in the CANSimple command set — use the board's own USB console"*. That is
the honest answer, not a stub:

| Command | Why |
|---|---|
| `KD` | `Set_Vel_Gains` has P and I fields only. |
| `JP` `JI` `JD` `J` | No CANSimple command touches the current loop. |
| `PI` `PD` | `Set_Pos_Gain` carries P only. |
| `H` | Hall calibration is a *blocking* commissioning sequence; the firmware's `rxSetAxisState` deliberately does not act on `ENCODER_OFFSET_CALIBRATION`. |

`Q` is the other asymmetry worth knowing about. **CANSimple has no configuration
read-back at all**, so the station cannot ask the board what it currently holds.
Its `cfg` line reports what it last *commanded* for the settable fields, and the
compile-time `CFG_*` constants for the rest — accurate unless something was also
changed over the board's own USB console, or the board is running a different
build. The line ends with `src=bridge` so a host can tell.

---

## The telemetry line

Emitted every `BRIDGE_TELEMETRY_MS` (100 ms, matching the board's `SER` task):

```
t=1200 #1 mode=2 tgt=5.00 Iq=0.42 vel=9.42 pos=12.57 Vbus=24.3 Irgn=1.25 RUN can_tx_ok=16 can_tx_fail=0 can_rx=4
```

The channels between `mode=` and the status word are generated from
[`../include/telemetry_schema.h`](../include/telemetry_schema.h). Two of them
are **omitted rather than faked**, because no CAN frame carries them:

| Channel | Why it is missing |
|---|---|
| `Ibrk` | Brake-resistor current is `duty × Vbus / R`, and the chopper duty is not published on CAN. Printing `0` would read as *"the brake never fires"* — the opposite of a diagnosis. |
| `blnd` | The hall/sensorless blend fraction is an internal of the observer. |

A channel is also withheld until its first frame has actually arrived, so an
early `0.0 V` is never mistaken for a dead bus.

The remaining fields:

| Field | Meaning |
|---|---|
| `tgt` | what **we sent**, in the unit of the active mode — a setpoint, not a reading |
| `RUN` / `idle` / `SAFE` | heartbeat fresh + closed loop + no error / fresh / no heartbeat |
| `[FAULT] err=0x…` | the heartbeat's axis error word |
| `can_tx_ok` `can_tx_fail` `can_rx` | **this station's** counters — the end of the link the board cannot see |

---

## Diagnostics

### Events go to the log; state goes to a page

The station used to print a line per transmitted frame, a nine-field counter
dump every two seconds, and a joystick line at 10 Hz, all interleaved with the
10 Hz telemetry. The web GUI's monitor pane scrolled faster than anyone could
read it, so the lines that mattered — a new error bit, a lost link, a rejected
command — went past unseen. **A log nobody can read is worse than no log: it
looks like diagnostics.**

The split that fixes it:

| | Goes to | Why |
|---|---|---|
| **Events** — something *changed* | the event log ([`lib/logging`](lib/logging/log.h)) | Edge-triggered, so it prints when a thing becomes true, not on every scan that observes it |
| **State** — counters, link, error words | the `can …` line → the GUI's **CAN Devices** page | Only meaningful as a live table; as prose it was the noise |

### The event log

```
log <ms> <sev> <tag> <free text>
```

`<ms>` is the station uptime (same clock as the telemetry line's `t=`), `<sev>` is
`E`/`W`/`I`/`D`, `<tag>` one of `SYS CAN BUS LINK AXIS POT`. The GUI
parses this ([`src/logevent.h`](../GUI/serial_plotter_wasm/src/logevent.h)),
colours by severity and lets you filter. Lines that do not match — the board's
own banner, every `AK …` acknowledgement — are still shown, as Info.

Four rules keep it readable, and they are the whole design:

1. **Levels.** `D0` errors, `D1` +warnings, `D2` +state changes (**default**),
   `D3` +per-frame CAN trace. The trace is off until asked for. This is a
   *source-side* filter: at `D2` the trace is never transmitted at all.
2. **Edges, not states.** Anything true for a while is reported through
   `logx::OnChange<>`, so a heartbeat repeating the same axis state 10× a second
   produces nothing.
3. **Periodic data is not logging.** Counters go on the `can …` line.
4. **Nothing gets through uncapped.** Identical consecutive messages fold into
   `previous line repeated N times`; a token bucket
   (`LOG_MAX_LINES_PER_S`) caps the whole stream. Errors are exempt from the
   cap — dropping the one line that says what broke would defeat the purpose.

What you actually see, at the default level:

| Line | Means |
|---|---|
| `log I CAN node 3 seen on bus (target)` / `log W CAN node 3 … NOT the target (0)` | Node-id mismatch, in one line at the first frame, instead of "no telemetry" with no explanation |
| `log I LINK established with node 0` / `log W LINK lost … drop #3; worst gap while up …` | The timeout is `BRIDGE_HEARTBEAT_MISSES ×` the firmware's own broadcast period, read from its table. **The message names the cause** — see [Why the link keeps dropping](#why-the-link-keeps-dropping) |
| `log I AXIS state IDLE -> CLOSED_LOOP` | Decoded from the shared [`axis_vocab.h`](../include/axis_vocab.h) |
| `log E AXIS error 0x0 -> 0x140 [MOTOR_FAILED\|ENCODER_FAILED]` | **A new error bit.** Decoded from the same table; a bit this build has no name for is shown as `+0xNNN` rather than dropped |
| `log W BUS error counters climbing while state=RUNNING …` | **The important one.** Errors counted while the controller still reports RUNNING is the fingerprint of a marginal link — typically a missing terminator — that is dropping frames without going bus-off. Reported once, on the transition |
| `log E BUS BUS-OFF: … recovering` | Too many errors; the controller disabled itself. Recovery is automatic |
| `log E CAN TX REJECTED id=0x00D CMD_SET_INPUT_VEL …` | The command you just issued did not leave the building. Always shown, at every level |
| `log D CAN TX id=0x00D … 4576743F00000000` | The frame trace, at `D3` only |

Command names in the trace are generated from the firmware's table, so an
`UNKNOWN` there is informative: that frame is not part of *this board's* command
set.

### Why the link keeps dropping

A link that drops repeatedly has three causes that need three different fixes,
and the bus counters alone cannot tell them apart — two of them leave every
counter clean. These are the fields that separate them, on the `can …` line and
on the CAN Devices page:

| What you see | Cause | Fix |
|---|---|---|
| `bus_ec` / `rx_ec` / `tx_ec` climbing | Frames being **corrupted** on the wire | Termination (120 Ω at *both* ends), CANH/CANL swap, common ground, bit-rate mismatch |
| `scan_max` approaching `hb_period` | **This station's loop is not keeping up.** Not a bus fault at all | The frame trace at `D3` is the usual cause — 130 frames/s of tracing is more than 115200 baud carries. Drop to `D2`, or raise `BRIDGE_SERIAL_BAUD` |
| `log W SYS loop stalled …` | The same thing, past `BRIDGE_SCAN_STALL_MS` | As above, or a serial host that stopped reading |
| `log W AXIS state CLOSED_LOOP -> IDLE - NOT commanded here` | **The board disarmed itself.** Nothing on this end did it | Its own console: a fault, or `CFG_WATCHDOG_MS` if a CAN setpoint timeout is configured |
| `rx_missed` / `rx_overrun` climbing | Frames arriving but **this end** is not draining fast enough | Same as above — the stall exceeded `BRIDGE_SCAN_STALL_MS` long enough to overflow the RX queue |
| `hb_max` ≈ `hb_period`, then a drop | The link was **clean right up to the moment it stopped**: the sender died | Board reset, watchdog, bus physically cut, node powered down |
| `hb_max` well above `hb_period` (say 300 ms against 100) | Frames were **already being lost** before the drop | Same list as the first row — but it also means `BRIDGE_HEARTBEAT_MISSES` is tighter than this bus deserves |

`hb_max` is the worst gap actually observed **between two heartbeats while the
link was up**. It can never reach `hb_timeout`, because exceeding that is what
declares the loss — which is exactly what makes it diagnostic: a link sitting at
the heartbeat period was healthy until it wasn't, and one sitting at 3–4× the
period was limping the whole time. `drops` counts link losses since boot, so
"very often" becomes a rate.

The last row is the one worth acting on before reaching for a soldering iron:
if `hb_max` is consistently high but well under the timeout, the link works and
`BRIDGE_HEARTBEAT_MISSES` (default 5, so 500 ms) is simply too strict. Raising
it to 8–10 costs nothing except a slower reaction to a genuine loss.

### The `can …` line

One line a second, key/value, hex where an operator reads hex:

```
can node=0 link=1 hb_age=8 hb_period=100 hb_timeout=500 hb_max=104 drops=0
    scan_max=2 stop_after=3000 stopped=0 bus=1 axis=8 mode=2
    axis_err=0x0 motor_err=0x0 enc_err=0x0 ctrl_err=0x0
    tx_ok=124 tx_fail=0 rx=2451 tx_ec=0 rx_ec=0 tx_failed=0 rx_missed=0
    rx_overrun=0 arb_lost=0 bus_ec=0 baud=500000 nodes=0x…01 loglvl=2
```

The GUI routes it to the **CAN Devices** page and never to the monitor pane. It
keeps the values as text, so `axis_err=0x140` survives as written and is decoded
against the same `axis_vocab.h` table this station uses. `nodes` is a bitmask of
every node id seen on the bus, which is how the page lists devices nobody
configured it to expect.

---

## How to add something

**A telemetry channel** — add it to the firmware's
[`telemetry_schema.h`](../include/telemetry_schema.h). This project then fails
to link until `bridge_telemetry.cpp` gains a `channel::yourkey` accessor.
Return `true` with the value if CAN carries it, `false` if it does not.

**A CAN command the board should accept** — one line in
[`can_commands.h`](../include/can_commands.h) plus a handler in
`lib/odrive_can/odrive_can.cpp`. Only then will `Axis::send<CMD>()` compile here.

**A console command shared with the board** — one line in
[`console_commands.h`](../include/console_commands.h) plus handlers on *both*
sides. If it has no CANSimple equivalent, call `notOverCan()` — that is a
complete answer, and the point of forcing the decision.

**A command only this station has** — one line in
[`bridge_commands.h`](lib/can_bridge/bridge_commands.h) plus a handler in
`bridge_console.cpp`. Pick a key that is not in the firmware's table.

**Anything that touches a payload's bytes** goes in `bridge_axis.cpp`, next to
the conversion it belongs with. It is the mirror of `odrive_can.cpp` on the
board; the two are halves of one encoding and must change together.

---

## Things that look arbitrary but are not

- **The RX queue is 32, not the default 5.** The board's cyclic timers
  (heartbeat, encoder, Iq, Vbus) all fire together on their first crossing, so
  it bursts 4+ frames in one millisecond at boot — while `setup()` here is still
  blocking on serial and has not started draining.
- **`poll()` drains the *whole* RX queue**, not one frame per pass. One frame
  per pass cannot keep up with a 100 Hz broadcast plus everything else.
- **The potentiometer has no spike filter, deliberately.** A spring-return pot
  snapping back from full deflection crosses a couple of thousand ADC counts
  inside one 100 ms poll. Rejecting that as a spike leaves the reference stale,
  so every subsequent legitimate reading near rest *also* looks like a huge jump
  and is rejected too — permanently latching the last velocity sent. That was
  observed: the target stuck at ±10 rad/s and never returned to 0.
- **The two sides of the pot's travel are scaled independently.** Its rest point
  is not the electrical mid-point, so a single symmetric scale makes one
  direction unusable.
- **The joystick does not re-assert velocity mode when already in it.** Doing so
  would double the bus traffic every tick, and would stop a one-off `T`/`X`
  command from ever holding.
- **`begin()` sends the limits but not the gains.** The limits are the
  firmware's own `CFG_*` values, so that send is a confirmation the station
  cannot arm against a limit the board was not configured for. The gains are
  left alone because the board already booted with them, and resending would
  stamp on a live tuning session running over its USB console.
- **`arm()` clears errors and zeroes the setpoint *before* closing the loop.**
  The other order arms onto whatever setpoint was left from the last session.
- **One clock per scan, passed down, never re-read.** `poll()` samples `millis()`
  once and threads it through the RX decoders. Reading the clock inside a
  decoder instead cost a full debugging session: draining and tracing the queue
  takes real milliseconds, so a heartbeat stamped with its own `millis()` landed
  *after* the `now` the timeouts were checked against. `now - last_heartbeat_ms`
  underflowed to ~4.29 billion, every timeout fired at once, and the station
  disarmed the motor while heartbeats were streaming in at a perfect 100 ms.
  `Axis::heartbeatAge()` now floors the subtraction at zero as well, because an
  age is a duration and durations are not negative — but the real fix is the
  single clock.
- **The safety stop latches BEFORE it transmits.** The transmit is likely to
  fail — the bus is the thing that just died — and retrying on failure would
  reproduce the command flood the one-shot exists to prevent, at exactly the
  moment somebody is trying to use the console.
- **The safety stop disarms rather than commanding zero velocity.** Holding
  0 rad/s keeps the axis armed and actively braking; `IDLE` lets it coast.
- **Boot calibration checks the SPREAD, not the value.** Comparing against the
  ohm-meter estimate would be checking a measurement against a guess. The spread
  is self-contained: a pot held still reads within a few counts, one being moved
  reads hundreds apart, and a disconnected pin reads wider than either.
- **The joystick logs at DEBUG, not as an acknowledgement.** It moves
  continuously and its value is already on every telemetry line as `tgt`; at
  INFO it was ten lines a second restating what the plot was drawing.
- **Bus alerts are not filtered, they are folded.** On a marginal link they fire
  thousands of times a second. Suppressing them would hide the fault; letting
  them through would hide everything else. `previous line repeated 4213 times`
  *is* the diagnosis.
- **ERROR lines are exempt from the rate cap.** The cap exists to keep the log
  readable, and the one line that says what broke is the one it is being kept
  readable for. An error storm is bounded by the folding instead.

---

## See also

- [`../README.md`](../README.md) — the firmware repository
- [`../src/README.md`](../src/README.md) — firmware architecture (modules over four tasks), and the tables this project consumes
- [`../docs/Getting_Started.md`](../docs/Getting_Started.md) — flashing and driving the board
- [`../docs/GUI.md`](../docs/GUI.md) — the web plotter / PID tuner that talks to either port
- [`../CAN/`](../CAN/) — the CANSimple DBC generator, and minimal send-only sketches
