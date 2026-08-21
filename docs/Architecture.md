# Firmware architecture — reference

The full picture: how the firmware is put together, what runs when, how modules
share values safely without a single mutex, and how to change any of it.

> 🚀 **New here?** Read [`src/README.md`](../src/README.md) first — it gets you
> productive in ten minutes. This page is the reference you come back to.

---

## Table of contents

1. [The shape of it](#1-the-shape-of-it)
2. [Boot, step by step](#2-boot-step-by-step)
3. [The four tasks](#3-the-four-tasks)
4. [Shared state](#4-shared-state)
5. [Concurrency and memory model](#5-concurrency-and-memory-model)
6. [Timing budget](#6-timing-budget)
7. [The modules](#7-the-modules)
8. [The I/O layer](#8-the-io-layer)
9. [The declaration tables](#9-the-declaration-tables)
10. [Walkthroughs](#10-walkthroughs)
11. [The DC-bus safety ladder](#11-the-dc-bus-safety-ladder)
12. [Making changes](#12-making-changes)
13. [Compile-time configuration](#13-compile-time-configuration)
14. [Verifying a change](#14-verifying-a-change)

---

## 1. The shape of it

Three layers and one rule.

```
        src/boot.cpp — hardware bring-up order, and the four tasks
        ┌──────────────┬──────────────┬──────────────┬──────────────┐
        │  SAFE 1 kHz  │  FOC 20 kHz  │ COMMS 1 kHz  │  SER 10 Hz   │
        │  prio 5      │  prio 4      │  prio 3      │  prio 2      │
        └──────┬───────┴──────┬───────┴──────┬───────┴──────┬───────┘
               │              │              │              │
        src/app/ — the control modules, one .cpp each
          safety.cpp      foc.cpp      comms.cpp      console.cpp
                                       control.cpp
                                       calibration.cpp
               │              │              │              │
               └──────────────┴──────┬───────┴──────────────┘
                                     │
                            src/state.h — the values
                            modules share, grouped by
                            WHO IS ALLOWED TO WRITE THEM
                                     │
                            src/io/ — the ONLY code that
                            touches a register, pin or peripheral
```

**A module** is one `.cpp` under `src/app/` plus two lines in
[`src/app.h`](../src/app.h). It owns its state as file-statics, reads and writes
`state.h`, and calls `io/` for hardware. It never creates a task and never
blocks — with one marked exception (§7.6).

**A task** is a function in [`boot.cpp`](../src/boot.cpp) that calls modules in
order. Adding a module to a task is one line.

**The rule** is in §5, and it is the reason there is no locking anywhere.

`main.cpp` is four lines of code: it calls `boot::run()`, which never returns.

### What is deliberately *not* here

There is no framework, no base class, no registry and no task table. Six modules
and four tasks did not justify the indirection: three of the four tasks run
exactly one module, and the one that runs three expresses its ordering as three
statements you can read in sequence.

---

## 2. Boot, step by step

All of it single-threaded, in [`src/boot.cpp`](../src/boot.cpp):

```cpp
void run() {
  bringUpHardware();      // 1. peripherals, in a required order
  app::control::init();   // 2. module cold start
  createTasks();          // 3. one xTaskCreate per task
  Serial.println("SAFE state (disarmed). ...");
  app::console::printBanner();
  vTaskStartScheduler();  // 4. never returns
}
```

### 2.1 `bringUpHardware()` — the order is load-bearing

| # | Call | Why here |
|---|---|---|
| 1 | `io::brake::preInit()` | AUX gates LOW **before** anything switches those pins to an alternate function, so the half-bridge never passes through an undefined state at power-up |
| 2 | `io::gate::preInit()` | DRV8301 GPIO + reset pulse. Before `Serial` because the DRV needs its 50 ms settling anyway, and the power stage should reach a known state as early as possible |
| 3 | `analogReadResolution(12)` | Board-global, owned by no single module; both the current sense and `io_vbus` assume it |
| 4 | `Serial.begin()` + banner | Waits up to 2 s for USB CDC enumeration, then gives up rather than hanging headless |
| 5 | `io::gate::init()` | DRV8301 over SPI3: out of reset, amplifier gain |
| 6 | `io::motor::init()` | Sensor → driver → current sense → motor parameters. Sets `state::at_boot.isense_ok` |
| 7 | `io::brake::init()` | **After** the DRV reset: GVDD powers the LM5109B, and only exists once the DRV8301 is awake |
| 8 | `io::vbus::init()` | **After** `motor::init()`: it takes whichever ADC the current sense did not claim |
| 9 | banner: brake/regen/OV thresholds | Prints the actual ladder in force, so a mis-set threshold is visible at boot |
| 10 | `io::can::init()` | Node id, bit rate, IRQ priority (see §10.3) |

### 2.2 Cold start before tasks exist

`app::control::init()` seeds the axis limits and gains from `motor_config.h`.
**This must run before any task is created.** If a COMMS scan ran first it would
push a zero current limit, which `updateSetpoint()` skips, leaving the axis on
the SimpleFOC default for a scan.

Only modules that need cold start have an `init()`; the rest start from their
file-static initialisers.

### 2.3 The banner between creation and start

Tasks are created, then the banner prints, then the scheduler starts. The gap is
deliberate: it is still single-threaded there, so the banner cannot interleave
with a telemetry line.

---

## 3. The four tasks

Declared in `createTasks()` in [`boot.cpp`](../src/boot.cpp); the numbers come
from [`include/config/tasks_config.h`](../include/config/tasks_config.h).

| Task | Trigger | Rate | Prio | Stack | Runs |
|---|---|---|---|---|---|
| `SAFE` | cyclic | 1 kHz | 5 | 512 w | `safety::update()` |
| `FOC` | **TIM6 event** | 20 kHz | 4 | 768 w | `foc::update()` |
| `COMMS` | cyclic | 1 kHz | 3 | 768 w | `comms::readFieldbus()` → `control::update()` → `comms::publishTelemetry()` |
| `SER` | cyclic | 10 Hz | 2 | 512 w | `console::update()` |

Stacks are in **words** (4 bytes), as `xTaskCreate` wants.

### 3.1 Cyclic tasks

```cpp
void safetyTask(void *) {
  TickType_t last = xTaskGetTickCount();   // baseline BEFORE the loop
  for (;;) {
    app::safety::update();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SCAN_MS_SAFETY));
  }
}
```

Body first, then `vTaskDelayUntil` against a baseline captured **before** the
loop. Delaying until an absolute baseline is what holds a 1 kHz task at 1 kHz
regardless of how long a scan took; a plain `vTaskDelay` lets the period drift
by the execution time.

### 3.2 The FOC event task

A 1 ms RTOS tick is useless at 20 kHz, so the FOC task is driven by a hardware
timer and a direct-to-task notification:

```cpp
void focTask(void *) {
  s_foc_task = xTaskGetCurrentTaskHandle();     // (1) BEFORE the timer runs

  HardwareTimer *tim = new HardwareTimer(TIM6);
  tim->setOverflow(FOC_TICK_HZ, HERTZ_FORMAT);
  tim->attachInterrupt(focTimerIsr);
  tim->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);   // (2)
  tim->resume();

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    // (3)
    app::foc::update();
  }
}
```

Three details, all of which compile perfectly when wrong and fail only at run
time:

1. **The handle is published before `resume()`.** The ISR fires as soon as the
   timer runs; publish afterwards and there is a window where notifications are
   dropped — intermittent, boot-only, and it looks like nothing at all.
2. **`NVIC_PRIO_RTOS_SAFE` (6).** The ISR calls `vTaskNotifyGiveFromISR`, so it
   must sit numerically **≥** `configMAX_SYSCALL_INTERRUPT_PRIORITY` (5).
   `configASSERT` is enabled, so getting this wrong is a hard hang on the first
   tick rather than silent kernel corruption — loud, but you need to know to
   look here.
3. **`pdTRUE` clears the count.** A tick missed while this task was preempted is
   *lost*, not made up later. That is correct for a control loop — one late
   execution beats two back-to-back with a bogus dt — and it is also why
   anything that steals CPU from this task at a fixed rate shows up as a
   periodic disturbance rather than as jitter (§6).

The ISR is a plain non-capturing function reaching its task through a static
pointer. A capturing lambda would go through `std::function`, which heap
allocates and adds an indirection inside a 20 kHz interrupt.

---

## 4. Shared state

Everything shared between modules is in [`src/state.h`](../src/state.h),
**grouped by who writes it**:

```cpp
namespace state {
  extern FromFoc     foc;       // writer: app::foc
  extern FromSafety  safety;    // writer: app::safety
  extern FromControl control;   // writer: app::control
  extern AtBoot      at_boot;   // written once during boot
  extern Requests    req;       // set by anyone, consumed by one
  extern odcan::AxisIO axis;    // multi-writer BY DESIGN
}
```

The grouping *is* the documentation: the struct name says who may write it, so a
reviewer sees a violation without checking a comment.

### `state::foc` — writer `app::foc`, published at 1 kHz

| Field | Unit | Notes |
|---|---|---|
| `shaft_angle` | rad | multi-turn, **sensor** convention (not sign-corrected) |
| `shaft_vel` | rad/s | sensor convention |
| `p_elec` | W | `Vq·Iq + Vd·Id`, computed by the writer — see below |
| `iq_measured` | A | |
| `iq_setpoint` | A | |

`p_elec` exists because those four dq terms are updated at 20 kHz. Reading them
from another task is a torn four-field read; the module that owns them computes
the product where it is coherent and publishes one atomic float.

### `state::safety` — writer `app::safety`

| Field | Unit | Notes |
|---|---|---|
| `vbus_filt` | V | median-of-3 then LPF, at `CFG_BUS_SAFETY_HZ` |
| `regen_iq_limit` | A | max &#124;Iq&#124; allowed to *oppose* rotation |
| `fault` | — | the latched fault. Set **and cleared** only here |

### `state::control` — writer `app::control`

| Field | Unit | Notes |
|---|---|---|
| `active_target` | rad/s, rad, A or V | follows the control mode |
| `foc_ready` | — | closed loop running |
| `calibrated` | — | `initFOC()` has succeeded once |

### `state::at_boot` / `state::req`

`at_boot.isense_ok` is written once during `io::motor::init()` and read-only
after. False means the voltage-torque fallback, which changes the units of every
gain and setpoint downstream.

`req.hall_cal` is a **request flag**: set by `console`, consumed and cleared by
`control`. Request flags are the one category the single-writer rule does not
cover — safe for a `bool` because both sides only ever store, never
read-modify-write.

### `state::axis` — the fieldbus block

[`include/axis_io.h`](../include/axis_io.h). Multi-writer by design: the CAN
driver, the console and `control` all command the axis. That is why it is
deliberately not one of the structs above.

Its one dangerous field has accessors:

```cpp
state::axis.raiseError(odcan::ERR_DC_BUS_OVER_VOLTAGE);
state::axis.clearErrorBits(seen);
```

**Never write `axis_error` with `|=`.** It is raised from tasks at different
priorities, and `|=` on a volatile is LDR/ORR/STR — a bit raised by the
preempting side between the load and the store is silently lost, and the losing
side is always the safety side. `raiseError()` uses `__atomic_fetch_or`, which
GCC inlines to LDREX/STREX on Cortex-M4.

`clearErrorBits(mask)` clears only the bits the caller **observed**, so a fault
raised after the caller's snapshot survives the clear.

`motor_error`, `encoder_error` and `controller_error` are **reserved and always
zero**. This firmware reports every condition through `axis_error`, for which
`axis_vocab.h` defines names all three projects share. They exist so the ODrive
`Get_*_Error` frames are well-formed and decode as "no further detail".
Populating them needs a sub-error vocabulary in `axis_vocab.h` first — a
three-repo table change, not a firmware change.

### The `req_*` flag protocol

```cpp
if (AX.req_vel_gains) { AX.req_vel_gains = false; /* act */ }
```

**Clear first, unconditionally, then act — and always report.** Clearing only on
the success path is how a command gets stuck looking dead. A console write
landing between the test and the clear is dropped; benign for idempotent gain
reapplication, which is what these are.

---

## 5. Concurrency and memory model

**There are no mutexes, no queues and no critical sections between modules.**
That is safe only because of one rule, and it is a correctness requirement, not
documentation:

> **Every shared variable has exactly one writer, and the struct it lives in is
> named after that writer.**

On Cortex-M4 a naturally-aligned 32-bit load or store is atomic. So a `float` or
`uint32_t` with a single writer can be read from any task without
synchronisation: a reader sees either the old value or the new one, never a
mixture.

**Pairs of variables are not atomic**, and this is the concrete bug the rule
exists to prevent. `Sensor::getAngle()` reads `(full_rotations, angle_prev)` — a
pair updated at 20 kHz. Calling it from the telemetry path returned torn values,
which appeared as **±1 turn spikes in `pos_rev`**. The fix is the design you
see: `app::foc` is the *only* code that touches the sensor, and it publishes
independent floats that everyone else reads.

The same argument produced `state::foc.p_elec` (§4) and moved the clear-errors
handling into `app::safety` (§7.2).

### Rules when adding shared state

| Do | Don't |
|---|---|
| Put it in the struct named for its writer | Write the same variable from two modules |
| Keep it to a single 32-bit word | Share a struct, an array, or a 64-bit value without protection |
| Mark it `volatile` | Rely on `volatile` for atomicity — it prevents caching in a register, nothing more |
| Derive multi-field state in one place and publish the result | Publish two fields a reader must combine |

If you genuinely need a multi-word atomic update, do not invent a lock: publish
a single derived value instead, or hand the whole computation to one module.

### Interrupt context

Only two ISRs exist in firmware code: the FOC tick trampoline
(`vTaskNotifyGiveFromISR`) and the hall sensor edge handlers
(`doHallA/B/C` → `HallSensor::handleX`, no RTOS call). The current-sense ADC ISR
belongs to SimpleFOC and makes no FreeRTOS call, so it may stay more urgent than
`NVIC_PRIO_RTOS_SAFE`. **Any new ISR that calls a `*FromISR` API must be set to
`NVIC_PRIO_RTOS_SAFE` or less urgent.**

### Heap

Two `new HardwareTimer(...)` allocations (FOC tick, brake PWM), both at boot,
never freed. FreeRTOS task stacks come from its own heap. Nothing allocates at
runtime, and it should stay that way — `configUSE_MALLOC_FAILED_HOOK` is live
and will halt with a message if that changes.

---

## 6. Timing budget

At `FOC_TICK_HZ` = 20 kHz, `foc::update()` has **50 µs per tick**. Everything
else must fit in what is left.

| Task | Period | Prio | Cost per scan | Notes |
|---|---|---|---|---|
| `SAFE` | 1 ms | 5 | ~1 µs, or ~25 µs every 5th | The ADC read is the spike |
| `FOC` | 50 µs | 4 | the bulk of the budget | `loopFOC()` + `move()` every 20th |
| `COMMS` | 1 ms | 3 | small, but `Serial.print` on gain changes is not | |
| `SER` | 100 ms | 2 | a whole telemetry line | Lowest priority, so it yields |

### The lesson that cost the most bench time

`safety` runs **above** `foc`. `io::vbus::readRaw()` is a *blocking* ADC
conversion of ~24 µs. Calling it every 1 ms steals ~24 µs from a 50 µs FOC
budget, at a perfectly fixed rate. Since notifications are not queued, the
stolen tick is lost rather than deferred.

The symptom was **a ~5 Hz velocity oscillation that appeared the moment
regenerative braking was added**. Not noise, not jitter — a periodic beat, from
a periodic theft.

The fix is the rate split inside `safety::update()`: nFAULT at 1 kHz (a free
digital read), the chopper at 1 kHz (two register writes — this accelerates the
*cut*, not the regulation), and the ADC + protection ladder at
`CFG_BUS_SAFETY_HZ` (200 Hz, since nothing on a battery bus has 1 kHz dynamics).

> **Before you add work to `SAFE` or any task above `FOC`, ask what it costs per
> scan and how often it truly needs to run.** Blocking calls in a
> higher-priority task are the failure mode to watch for, and they show up as
> control-loop artefacts, not as an error.

---

## 7. The modules

Declared in [`src/app.h`](../src/app.h), one `.cpp` each in `src/app/`.

### 7.1 `foc` — 20 kHz, event-paced

The hard real-time path. Nothing here may block, allocate or print.

- Below `foc_ready`, or while faulted, it only refreshes the sensor so pos/vel
  stay live.
- Otherwise `loopFOC()` then `move(state::control.active_target)`.
- Then the **regen clamp**: bound only the torque that *opposes* rotation
  (`sp · vel < 0`). Motoring torque is never reduced — it does not charge the
  bus, so limiting it would cost performance for nothing. Applied *after*
  `move()` because `current_sp` persists between two `move()` calls
  (`MOTION_DOWNSAMPLE` = 20), so clamping after covers the following
  `loopFOC()` calls too.
- Every 20th tick (1 kHz) it publishes `state::foc` — see §4.

> It never writes `motor.shaft_angle` / `shaft_velocity`. `move()` stores the
> filtered multi-turn angle there; overwriting mixes two reference frames into
> the telemetry.

### 7.2 `safety` — 1 kHz, top priority

Owns the fault latch outright. Three rates in one module, each deliberate — see
§6 for why. Runs the ladder in §11.

It also **consumes `req_clear_errors`**, which is the reason it is the only
writer of `state::safety.fault`. If another task cleared the latch, a
clear-errors serviced there could erase a fault latched microseconds earlier and
re-arm into a live over-voltage — the console runs at 100 ms and comms at 1 ms,
so a clear is always at least one safety scan behind what it is clearing. There
is no window here: a still-true condition re-latches further down the same call.

### 7.3 `control` — 1 kHz

The axis state machine, in scan order:

1. reboot / clear-error forwarding (must work even while faulted)
2. pending gain updates (`req_*_gains`)
3. blocking commissioning sequences, which take the scan for themselves
4. arm / disarm, `initFOC()` on first arm
5. the runtime setpoint, ramped

The velocity setpoint is **ramped**, not stepped: a step (V5 → V10) makes a
bench supply dip then overshoot, and the regen spike can trip the bus. The
ramp's state *is* `state::control.active_target` — zeroed on disarm and written
directly by the position and torque modes — so a helper holding its own copy
would go stale across those transitions and a re-arm would resume slewing from a
setpoint nobody asked for.

### 7.4 `comms` — 1 kHz, split around `control`

Pure transport, and deliberately so: it reads `state.h` and never touches the
motor object.

- `readFieldbus()` drains CAN into `state::axis`.
- `publishTelemetry()` publishes the axis block and runs the cyclic CAN TX.

The split is the contract: a setpoint arriving in a given millisecond is acted on
and reported in that same millisecond, rather than one scan later.

### 7.5 `console` — 10 Hz

Command dispatch (generated from
[`console_commands.h`](../include/console_commands.h), §9) and one telemetry
line per scan (generated from
[`telemetry_schema.h`](../include/telemetry_schema.h)).

> ⚠️ Both line formats are **parsed by other software** — the web GUI and the
> ESP32 CAN station. Changing a key, a decimal count or an `AK` prefix breaks
> them silently. See §14.

### 7.6 `calibration` — blocking, on request

The deliberate exception to "modules never block": hall calibration takes ~10 s,
R/L characterisation a few seconds. Both drive the phases directly and require
the motor **disarmed**.

Tolerable only because `control` runs in COMMS, *below* safety and FOC in
priority, so the motor stays protected throughout. But **CAN RX is not drained
for the duration** and a master will see the node stop answering. Do not call a
sequence from a higher-priority task.

Four conditions any blocking sequence must satisfy:

1. It runs only while disarmed and healthy.
2. It consumes its request flag unconditionally, and reports every refusal.
3. It leaves the stage disabled on exit.
4. It runs in the lowest-priority task that can host it.

---

## 8. The I/O layer

`src/io/` is the **only** code allowed to touch a register, a pin or a
peripheral. That is what keeps peripheral ownership checkable in one place.

| Module | Owns |
|---|---|
| `io_motor` | The SimpleFOC object graph: driver, motor, sensor → smoothing → hybrid, current sense. Writes `state::at_boot.isense_ok` |
| `io_gate` | DRV8301 over SPI3, plus `EN_GATE` / `nFAULT` as inline accessors |
| `io_brake` | TIM2 CH3/CH4 centre-aligned complementary PWM with dead time in ticks |
| `io_vbus` | The dedicated Vbus ADC — a blocking one-shot, ~24 µs |
| `io_can` | The `OdriveCAN` instance bound to `state::axis`. Three lines of substance |
| `io_console` | Serial line assembly and the `AK …` acknowledgement format |

`HallSensorSmoothVel.h` and `HybridSensor.h` are SimpleFOC `Sensor`
implementations rather than I/O modules; they live here because they are the
sensor's business.

### The output/logic split

Modules decide, `io/` acts. `safety` computes a duty; `io::brake::setDuty()`
programs TIM2. Keeping the decision out of the driver is what lets the ladder be
reasoned about without reading register code.

---

## 9. The declaration tables

Four X-macro tables in `include/` are compiled by **more than one project**. Add
a line and every derived artefact updates.

| Table | Expanded by |
|---|---|
| [`console_commands.h`](../include/console_commands.h) | `app/console.cpp` (dispatch + banner) **and** `can_utilities` |
| [`can_commands.h`](../include/can_commands.h) | `lib/odrive_can/` (×4) **and** `can_utilities` |
| [`telemetry_schema.h`](../include/telemetry_schema.h) | `app/console.cpp`, the **web GUI**, and `can_utilities` |
| [`axis_vocab.h`](../include/axis_vocab.h) | `axis_io.h` enums, `can_utilities` decoder, the GUI's CAN page |

Plus [`can_ids.h`](../include/can_ids.h), the CANSimple arbitration ids, shared
with `can_utilities`.

> ⚠️ **`include/` is a published interface.** Three hardcoded relative paths
> point into it — `can_utilities/platformio.ini`,
> `GUI/serial_plotter_wasm/CMakeLists.txt` and its `tests/CMakeLists.txt` — and
> the GUI includes `telemetry_schema.h` and `axis_vocab.h` by bare name. Do not
> move it.

### `telemetry_schema.h` has a firmware-private column

Its `expr` column names firmware symbols (`state::control.active_target`, …).
Both other consumers **discard** that argument, so editing it cannot break them —
and forgetting to update it is a firmware compile error, which is the good kind.
Its one constraint: `expr` must be a valid preprocessor argument — balanced
parentheses, **no top-level commas**.

---

## 10. Walkthroughs

### 10.1 A CAN velocity setpoint

```
CAN frame (0x00D, node<<5|cmd)
   → OdriveCAN::poll()                        [COMMS, via comms::readFieldbus]
       writes state::axis.input_vel  (rev/s → rad/s at the boundary)
       stamps state::axis.last_setpoint_ms
   → control::update()                        [same scan]
       clamps to min(vel_limit, CFG_VEL_CMD_MAX)
       ramps  → state::control.active_target
   → foc::update()                            [next 20 kHz tick]
       motor.move(state::control.active_target) → TIM1 PWM
   → comms::publishTelemetry()                [same COMMS scan as control]
       state::foc.* → state::axis.{pos_rev, vel_rev, iq_*, ibus}
```

The setpoint is acted on and reported in the millisecond it arrived, because
COMMS runs the three in that order.

### 10.2 An over-voltage event

```
safety::update()  [1 kHz]
  every 5th scan (200 Hz):
    io::vbus::readRaw()  →  median-of-3  →  LPF  →  state::safety.vbus_filt
      ├─ > OV_TRIP for 2 consecutive samples (5 ms sustained)
      │     → io::gate::disable(), state::safety.fault = true
      │     → state::axis.raiseError(ERR_DC_BUS_OVER_VOLTAGE)
      └─ regen derate → state::safety.regen_iq_limit  (read by foc)
  every scan (1 kHz):
    chopper hysteresis → io::brake::setDuty()
```

### 10.3 The CAN IRQ priority trap

`OdriveCAN::begin()` calls `setIRQPriority()` and `setAutoBusOffRecovery()`
**before** `_can.begin()`. Both are documented as setup-only.

Getting it wrong is silent. `preemptPriority` is never initialised by the
library's constructor, so calling `setIRQPriority()` *after* `begin()` left the
CAN1 ISR at NVIC priority 0 — **above** `configMAX_SYSCALL_INTERRUPT_PRIORITY` —
where it could preempt a FreeRTOS critical section mid-update inside the driver's
own ring buffers and corrupt them.

Symptom: *a few frames work, then everything wedges forever,
non-deterministically depending on boot timing.*

`begin()` also defaults `retransmission` to false despite its header comment, and
the constructor separately disables auto bus-off recovery — so a handful of
unacked frames at boot (before the peer's driver is up) could push the controller
into BUS_OFF, which then never recovers. Both are set explicitly.

---

## 11. The DC-bus safety ladder

Three stages, softest to hardest, all inside
[`app/safety.cpp`](../src/app/safety.cpp). **The threshold ordering in
[`motor_config.h`](../include/config/motor_config.h) is what makes it correct:**

```
BRAKE_VBUS_OFF < BRAKE_VBUS_ON < REGEN_START < REGEN_FULL < OV_TRIP
     24.2           24.6            26.5          27.5        29.0     (PSU)
```

| Stage | Action |
|---|---|
| 1 | **Chopper** — dissipate into the 2 Ω resistor, proportional duty above `BRAKE_VBUS_ON` |
| 2 | **Regen derate** — withdraw permission to brake electrically, linearly `REGEN_START → REGEN_FULL` |
| 3 | **OV trip** — latched fault: EN_GATE low, motor freewheels |

Because the chopper's thresholds sit **below** the derate's, the resistor always
gets its chance before any braking torque is sacrificed. Stage 3 is only reached
if 1 and 2 both failed. **Preserve that ordering if you retune.**

### Stage 1 detail: the hysteresis

Engage above `VBUS_ON`, release only on falling back below `VBUS_OFF`. Once
engaged the duty is computed from **`VBUS_OFF`, not `VBUS_ON`** — otherwise it
would be negative (hence zero) across the whole hysteresis band, and the chopper
would chatter around `VBUS_ON` instead of holding the bus inside it.

### Stage 3 detail: the debounce

Two consecutive 200 Hz samples = **5 ms of sustained** over-voltage, and up to
**10 ms of worst-case latency** from the crossing. Those are different numbers
and worth keeping apart: the sustained window rejects a transient, the latency is
what the bus capacitance has to survive.

A `static_assert` guards it. Lowering `CFG_BUS_SAFETY_HZ` erodes the debounce,
and at 100 Hz it would vanish entirely — the last-resort fault would latch on a
*single* ADC sample. `motor_config.h` only requires the rate to divide 1000, so
nothing else stops that edit.

### Two hardware facts that constrain this

**The brake cannot work while disarmed.** The LM5109B's VDD comes from GVDD, the
DRV8301's *internal* gate regulator, which only exists while EN_GATE is high.
This is board wiring, not policy. Consequences: cutting EN_GATE at stage 3 also
kills dissipation, so past `OV_TRIP` the motor simply freewheels (intended — the
fault is a last resort, not a regulation mode); and **there is no over-voltage
protection at all while disarmed**, e.g. a motor driven mechanically.

> The documented fix, if permanent protection is ever needed: hold EN_GATE high
> at all times and cut the motor via TIM1's `BDTR.MOE = 0`, which puts the six
> motor gates in Hi-Z without losing GVDD.

**The AUX bridge must be driven complementary, with dead time.** Driving one FET
does nothing: the low FET alone pulls the midpoint to ground (0 V across the
resistor); the high FET alone never turns on, because its bootstrap capacitor
only charges while the low FET conducts. Hence complementary drive — and hence
mandatory dead time, on pain of a hard bus short. `CFG_BRAKE_MAX_DUTY` < 1.0 is
mandatory for the same bootstrap reason.

---

## 12. Making changes

### Change a task's rate, priority or stack

[`include/config/tasks_config.h`](../include/config/tasks_config.h), one
`#define`. Nothing else needs to know.

⚠️ If you change a *rate*, check anything whose timing depends on it:
`util::Debounce` presets take their period as an argument, so they follow — but
the Vbus filter's fixed alpha is a **sample-count** filter, so retuning
`CFG_BUS_SAFETY_HZ` changes its time constant with it (see the note in
`safety.cpp`).

### Add a task

One `Spec` row in `createTasks()` plus a task body next to the others in
[`boot.cpp`](../src/boot.cpp). Take the priority and stack from
`tasks_config.h` rather than hard-coding numbers.

> ⚠️ Before giving anything a priority above `PRIO_FOC`, read §6. A blocking call
> in a higher-priority task shows up as a control-loop artefact, not as an error.

### Add an event (interrupt-paced) task

Copy `focTask()`. The three details in §3.2 are the whole difficulty: publish
the handle before starting the timer, set the ISR to `NVIC_PRIO_RTOS_SAFE`, and
use a non-capturing ISR function.

### Change the FOC rate

`FOC_TICK_HZ` in `tasks_config.h`. Check the 50 µs budget (§6) still holds, and
note that `foc.cpp`'s telemetry divider is `FOC_TICK_HZ / 1000`, so the 1 kHz
publish rate follows automatically.

### Change the sensor

`SENSOR_TYPE` in [`motor_config.h`](../include/config/motor_config.h)
(`SENSOR_TYPE_QUADRATURE` / `SENSOR_TYPE_HALL`), or override it in
`platformio.ini`. `io_motor.cpp` selects the sensor graph;
`console_commands.h` guards the hall-calibration command with the same macro.

> Build `can_utilities` with the **same** setting, or the two ends disagree about
> whether the `H` command exists.

### Replace or remove the fieldbus

`io_can` and `lib/odrive_can/` are the only code that knows CANSimple exists.
`app::comms` reads and writes `state::axis` and nothing else. Removing CAN means
deleting `comms::readFieldbus()`'s body and leaving the console as the only
master.

### Add a shared variable

Put it in the `state.h` struct named for its writer. If no existing struct fits,
the value probably has two writers — resolve that first (§5).

---

## 13. Compile-time configuration

| File | Change it when |
|---|---|
| [`config/hw_pinout.h`](../include/config/hw_pinout.h) | the board changes |
| [`config/motor_config.h`](../include/config/motor_config.h) | the motor or the tuning changes |
| [`config/tasks_config.h`](../include/config/tasks_config.h) | the schedule changes |
| [`board_config.h`](../include/board_config.h) | never — a three-line umbrella kept only so the standalone bench sketches in `test/` compile |

`motor_config.h` is also compiled by `can_utilities`, which takes the CAN node
id, bit rate and limit ceilings straight from it. Change a limit there and both
ends of the bus follow.

### The build flag you must not remove

```ini
build_flags = -I$PROJECT_INCLUDE_DIR
```

PlatformIO puts `include/` on the compiler path for `src/` but **not** for
`lib/` targets, and two things depend on it:

- `lib/odrive_can` includes `axis_io.h`, the axis block it is mapped onto;
- the FreeRTOS library's `__has_include("STM32FreeRTOSConfig.h")` probe. Without
  the flag it answers "no" and the kernel silently uses its own defaults instead
  of `include/STM32FreeRTOSConfig.h` — including leaving stack-overflow and
  malloc-failure detection switched **off**.

The second one failed silently for a long time. If those hooks ever stop firing,
check that flag first.

---

## 14. Verifying a change

1. **It compiles.** `pio run -e genericSTM32F405RG`.
2. **Flash and RAM did not jump.** Compare against the previous build. A large
   swing on a change you thought was mechanical means something was dropped or
   duplicated.
3. **`can_utilities` still compiles.** `cd can_utilities && pio run`. This is the
   check that proves you did not disturb the shared tables in `include/`.
4. **The GUI's parser tests pass.** They assert the `t=…`, `cfg …` and `AK …`
   grammars:
   ```
   cd GUI/serial_plotter_wasm/tests && qt-cmake -B build && ninja -C build && ./build/hub_test
   ```
5. **The boot banner is unchanged**, line for line, against a known-good capture.
   Each line marks a bring-up step, so a missing one localises the failure.
6. **On the bench:** `A` arms, `V5` spins, `I` idles; the telemetry line arrives
   at 10 Hz with the same fields; the chopper still engages when you drive the
   bus up; `H` and `M` still complete.
7. **Stack headroom**, if you added anything to a task:
   `uxTaskGetStackHighWaterMark()` on all four.

### Where to look when something breaks

| Symptom | Look at |
|---|---|
| A task halts with a named stack-overflow message | `STACK_*` in [`tasks_config.h`](../include/config/tasks_config.h) — the check is live and names the task |
| Periodic oscillation at a task's rate | Something blocking in a task above `foc` — §6 |
| A few CAN frames then a permanent wedge | IRQ priority / bus-off recovery — §10.3 |
| ±1 turn jumps in position | A non-atomic pair read across tasks — §5 |
| A serial command does nothing | A `req_*` flag cleared only on the success path — §4 |
| Hard hang on the first FOC tick | The TIM6 NVIC priority — §3.2 |
| False over-voltage during alignment | ADC sharing — [`hw_pinout.h`](../include/config/hw_pinout.h) near `PIN_VBUS` |
| Boot stops partway | Diff the banner against a known-good capture — §2.1 |

---

## See also

- [`src/README.md`](../src/README.md) — the orientation guide: module map and
  how to add a feature
- [Getting_Started.md](Getting_Started.md) — flashing and driving the board
- [Calibration.md](Calibration.md) — commissioning a new motor
- [GUI.md](GUI.md) — the web plotter / PID tuner / CAN devices page
- [`can_utilities/README.md`](../can_utilities/README.md) — the ESP32 CAN
  control station, and the tables it shares with this firmware
