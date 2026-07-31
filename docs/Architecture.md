# Firmware architecture — reference

The complete internals of the firmware. Read [`src/README.md`](../src/README.md)
first — it orients you and covers the common "how do I add X" recipes. **This**
document is for when you need to change something *structural*: the runtime, the
process image, the scheduling, the I/O layer, or a safety path.

## Table of contents

1. [Why a PLC model](#1-why-a-plc-model)
2. [The execution model](#2-the-execution-model)
3. [Boot, step by step](#3-boot-step-by-step)
4. [The runtime internals](#4-the-runtime-internals)
5. [The process image](#5-the-process-image)
6. [Concurrency and memory model](#6-concurrency-and-memory-model)
7. [Timing budget](#7-timing-budget)
8. [Function blocks](#8-function-blocks)
9. [The I/O layer](#9-the-io-layer)
10. [The three declaration tables](#10-the-three-declaration-tables)
11. [Walkthroughs](#11-walkthroughs)
12. [The DC-bus safety ladder](#12-the-dc-bus-safety-ladder)
13. [Modifying core components](#13-modifying-core-components)
14. [Compile-time configuration](#14-compile-time-configuration)
15. [Verifying a change](#15-verifying-a-change)
16. [Glossary](#16-glossary)

---

## 1. Why a PLC model

This is a motor controller: a handful of loops at fixed rates, a lot of shared
state, and several paths that can destroy hardware if they run in the wrong
order. That is the exact problem IEC 61131-3 was designed around, and the
industry solved it with four ideas:

| Idea | What it buys here |
|---|---|
| **Fixed-period tasks** declared in one table | The schedule is readable in one place instead of inferred from scattered `xTaskCreate` calls |
| **Programs** that run to completion each scan | No hidden blocking, no reentrancy, no "which task is this called from?" |
| **Function blocks** with explicit inputs/outputs | Control logic separated from hardware, and testable without a board |
| **An explicit process image** | Every shared variable has a declared owner, which is what makes lock-free sharing provably safe |

The alternative — and what this code was before — is a single 1146-line
`main.cpp` where the clock config, the ADC driver, the state machine, four task
bodies and a 210-line command parser all sit together, and the layering exists
only as comments asking you not to cross it.

> **This is a model, not a compliance claim.** There is no ST/ladder compiler, no
> runtime POU loader, no online change. We borrow the *organisation* because it
> fits, not to tick a standard's boxes. Where the standard and the hardware
> disagree, the hardware wins — and the deviation is documented at the point it
> happens.

---

## 2. The execution model

### The hierarchy

```
CONFIGURATION                       src/config/configuration.cpp
 └── RESOURCE  (the STM32F405)      src/plc/plc_runtime.cpp
      ├── TASK "SAFE"   cyclic 1 ms,    prio 5
      │    └── PRG_SAFETY
      ├── TASK "FOC"    event 20 kHz,   prio 4
      │    └── PRG_FOC
      ├── TASK "COMMS"  cyclic 1 ms,    prio 3
      │    ├── PRG_FIELDBUS_IN
      │    ├── PRG_CONTROL
      │    └── PRG_TELEMETRY_OUT
      └── TASK "SER"    cyclic 100 ms,  prio 2
           └── PRG_CONSOLE
```

One `TaskDef` row becomes one FreeRTOS task. Higher priority number = more
urgent; the numbers are in [`include/config/plc_config.h`](../include/config/plc_config.h).

### The scan cycle

A task fires, runs each of its programs **once, in the order declared**, then
waits for its next trigger. Nothing is interleaved *within* a task.

The `COMMS` task is where this matters most, and its program order is the
classic PLC scan made literal:

```
PRG_FIELDBUS_IN   →   PRG_CONTROL   →   PRG_TELEMETRY_OUT
  read inputs           run logic          write outputs
  (drain CAN RX)     (state machine)   (publish + send CAN)
```

Because of that ordering, a `Set_Input_Vel` frame that arrives during
millisecond *N* is applied to the motor in millisecond *N* and reported on the
wire in millisecond *N*. Reorder those three and you add a full scan of latency,
or publish stale telemetry. **If you add a program to `COMMS`, decide
deliberately whether it belongs before or after `PRG_CONTROL`.**

### Programs

```cpp
class Program {
 public:
  virtual void init() {}            // once, before the scheduler, in table order
  virtual void scan() = 0;          // once per task trigger
  virtual const char* name() const = 0;
};
```

The contract a program must honour:

| Rule | Why |
|---|---|
| Never create a task | The CONFIGURATION owns the schedule |
| Never touch a peripheral directly | That is `src/io/`'s job, and its exclusivity is what makes ADC/timer ownership analysable |
| Never block or delay | The task's trigger paces it; a delay inside a scan silently changes the rate of everything after it in the task |
| Hold FB state as members, not globals | Scopes the state to the program, and makes two instances possible |

`src/seq/` is the deliberate exception — see [§9](#the-blocking-exception-srcseq).

---

## 3. Boot, step by step

`setup()` calls `configuration::boot()` and never returns. **The order below is
load-bearing at almost every step.**

```cpp
void boot() {
  bringUpHardware();
  plc::createTasks(TASKS, TASK_COUNT);   // program init() + xTaskCreate
  Serial.println("SAFE state (disarmed). ...");
  prog::printConsoleBanner();
  plc::start();                          // vTaskStartScheduler, never returns
}
```

`bringUpHardware()`, in order:

| # | Step | Why *here* |
|---|---|---|
| 1 | `io::brake::preInit()` — AUX gates LOW as plain GPIO | Before anything switches those pins to an alternate function, so the half-bridge never passes through an undefined state at power-up |
| 2 | `io::gate::preInit()` — M1_CS high, nFAULT pull-up, EN_GATE LOW→HIGH pulse | The DRV8301 latches configuration on the EN_GATE rising edge; the low pulse guarantees a known starting state. Two 50 ms delays live here |
| 3 | `analogReadResolution(12)` | Board-global; both the current sense and `io_vbus` assume 12-bit |
| 4 | `Serial.begin()` + 2 s wait for host | Everything after this can report failures |
| 5 | `io::gate::init()` — DRV8301 over SPI, amplifier gain | Needs the reset pulse (2) done and Serial (4) up to report SPI failure |
| 6 | `io::motor::init()` — sensor, driver, current sense, motor params | Must follow (5): the current-sense gain must match the gain programmed into DRV8301 CTRL2 |
| 7 | `io::brake::init()` — TIM2 center-aligned, bridge STOPPED | Must follow (5): the LM5109B's VDD is GVDD, which only exists with the DRV8301 awake |
| 8 | `io::vbus::init()` | **Must follow (6)**: it claims whichever of ADC1/ADC2 the current sense did *not* take |
| 9 | Brake/threshold banner | Reports the configuration actually in force |
| 10 | `io::can::init()` | Last; nothing else depends on it |

Then `plc::createTasks()` calls `init()` on every program (this is where
`PRG_CONTROL` seeds the axis defaults from the configuration) before creating
any task — so no task can observe a half-initialised process image.

> Boot is also the cheapest end-to-end test you have. The banner exercises the
> DRV8301 SPI, ADC allocation, current-sense init, brake thresholds and CAN
> bring-up. **Diff it against a known-good capture after any structural change.**

---

## 4. The runtime internals

[`src/plc/plc_runtime.cpp`](../src/plc/plc_runtime.cpp) is ~150 lines. Both task
shapes reproduce the hand-written bodies they replaced.

### Cyclic tasks

```cpp
TickType_t last = xTaskGetTickCount();
for (;;) {
  for (uint8_t i = 0; i < t.program_count; i++) t.programs[i]->scan();
  vTaskDelayUntil(&last, pdMS_TO_TICKS(t.trigger.interval_ms));
}
```

`vTaskDelayUntil` against a baseline captured *before* the loop is what holds a
1 kHz task at 1 kHz regardless of how long a scan took. A plain `vTaskDelay`
would let the period drift by the execution time — and the drift would be
load-dependent, so it would look like jitter.

Note the order: **programs run first, then the delay.** A task's first scan
therefore happens immediately when the scheduler starts.

### Event tasks

```cpp
s_eventHandle[slot] = xTaskGetCurrentTaskHandle();   // FIRST
HardwareTimer *tim = new HardwareTimer(t.trigger.timer);
tim->setOverflow(t.trigger.event_hz, HERTZ_FORMAT);
tim->attachInterrupt(TRAMPOLINE[slot]);
tim->setInterruptPriority(NVIC_PRIO_RTOS_SAFE, 0);
tim->resume();
for (;;) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  for (...) t.programs[i]->scan();
}
```

Three things here are not free to reorder or simplify:

**The handle is published before the timer runs.** The ISR fires as soon as
`resume()` is called. Publishing the handle afterwards leaves a window where
notifications are dropped.

**The ISR is a static trampoline, not a lambda.** `HardwareTimer::attachInterrupt`
takes a `std::function`, so a capturing lambda would heap-allocate and add an
indirection *inside a 20 kHz interrupt*. Instead there is a fixed array of
handles and one `template <int SLOT> void eventTrampoline()` per slot, giving
the same generated code as a hand-written ISR. The count is
`PLC_MAX_EVENT_TASKS` (currently 2); `createTasks()` halts loudly at boot if the
table asks for more.

**The interrupt priority must be numerically ≥ 5.** The ISR calls
`vTaskNotifyGiveFromISR`, a FreeRTOS `*FromISR` API. STM32duino FreeRTOS derives
`configMAX_SYSCALL_INTERRUPT_PRIORITY` from library value 5, so any ISR calling
such an API must sit at a *less urgent* priority than that.
`NVIC_PRIO_RTOS_SAFE` is 6, giving margin. **Getting this wrong does not fail
loudly** — it corrupts kernel state under load. The same trap bit the CAN driver
(see [§11.3](#113-the-can-irq-priority-trap)).

**Notifications are not queued.** `ulTaskNotifyTake(pdTRUE, ...)` clears the
counter, so a tick that arrives while the task is preempted is *lost*, not made
up later. This is correct for a control loop — one late execution beats two
back-to-back with a bogus `dt` — but it is also why anything that steals CPU at
a fixed rate shows up as a periodic disturbance rather than as jitter. See
[§7](#7-timing-budget).

---

## 5. The process image

Types in [`include/gvl/gvl.h`](../include/gvl/gvl.h), instances in
[`src/gvl/gvl.cpp`](../src/gvl/gvl.cpp). **Everything shared between programs
lives here and nowhere else.**

### `gvl::IN` — %I, measured values

| Variable | Unit | Writer | Readers | Notes |
|---|---|---|---|---|
| `shaft_angle` | rad, multi-turn | `PRG_FOC` | `PRG_TELEMETRY_OUT` | Sensor convention, not yet sign-corrected |
| `shaft_vel` | rad/s | `PRG_FOC` | `PRG_TELEMETRY_OUT` | Published every 20th FOC tick (1 kHz) |
| `vbus_filt` | V | `PRG_SAFETY` | `PRG_SAFETY`, `PRG_TELEMETRY_OUT` | Median-of-3 + LPF, refreshed at 200 Hz |
| `isense_ok` | bool | `io::motor::init()` | everyone | Boot-time only. **Changes the units of every gain and setpoint downstream** |

### `gvl::Q` — %Q, commands from the logic

| Variable | Unit | Writer | Readers |
|---|---|---|---|
| `active_target` | rad/s, rad, or A/V per mode | `PRG_CONTROL` | `PRG_FOC`, telemetry |
| `regen_iq_limit` | A | `PRG_SAFETY` | `PRG_FOC` |

### `gvl::M` — %M, machine state

`fault`, `foc_ready`, `calibrated`, `req_hall_cal`. Written by `PRG_SAFETY`
(fault) and `PRG_CONTROL` (the rest), read widely.

### `gvl::AXIS` — the fieldbus-mapped block

`odcan::AxisIO`, defined in [`include/gvl/axis_io.h`](../include/gvl/axis_io.h).
Commands (armed, estop, control_mode, setpoints, limits, gain mirrors, request
flags) plus telemetry (pos/vel in **rev**, Iq, bus V/I, error, state).

It lives under `include/gvl/` rather than inside `lib/odrive_can` on purpose:
it is *process data*, not protocol. The CAN driver receives it by reference
(`explicit OdriveCAN(AxisIO&)`) and is otherwise completely decoupled — you can
delete the fieldbus without touching a line of control logic.

> **Units convention:** everything firmware-side is SI/rad. The rev and Nm
> conversions happen at the CAN boundary only. If you add a variable, keep it
> SI — a mixed-unit process image is how sign and scale bugs get in.

### The `req_*` flag protocol

Several commands cross tasks as a flag: `req_vel_gains`, `req_characterise`,
`req_clear_errors`, `req_reboot`, `req_hall_cal`. The convention is
**consumer-clears, and always clears**:

```cpp
if (AX.req_characterise) {
  AX.req_characterise = false;   // clear FIRST, unconditionally
  seq::motorCharacterise(safe);  // then act, and always report
  return;
}
```

Clearing only on the success path is how a request gets stuck set and the
command looks dead — which is exactly what an `M` sent while armed used to do.

---

## 6. Concurrency and memory model

**There are no mutexes, no queues and no critical sections between programs.**
That is safe only because of one rule, and it is a correctness requirement, not
documentation:

> **Every process-image variable has exactly one writer, named in a comment
> beside it.**

On Cortex-M4 a naturally-aligned 32-bit load or store is atomic. So a `float` or
`uint32_t` with a single writer can be read from any task without
synchronisation: a reader sees either the old value or the new one, never a
mixture.

**Pairs of variables are not atomic**, and this is the concrete bug the rule
exists to prevent. `Sensor::getAngle()` reads `(full_rotations, angle_prev)` — a
pair updated at 20 kHz. Calling it from `PRG_TELEMETRY_OUT` returned torn
values, which appeared as **±1 turn spikes in `pos_rev`**. The fix is the design
you see: `PRG_FOC` is the *only* code that touches the sensor, and it publishes
two independent floats that everyone else reads.

### Rules when adding shared state

| Do | Don't |
|---|---|
| One writer, named in a comment | Write the same variable from two programs |
| Keep it to a single 32-bit word | Share a struct, an array, or a 64-bit value without protection |
| Mark it `volatile` | Rely on `volatile` for atomicity — it prevents caching in a register, nothing more |
| Derive multi-field state in one place and publish the result | Publish two fields a reader must combine |

If you genuinely need a multi-word atomic update, do not invent a lock: publish
a single derived value instead, or hand the whole computation to one program.

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
runtime, and it should stay that way — `configUSE_MALLOC_FAILED_HOOK` is now
live and will halt with a message if that changes.

---

## 7. Timing budget

At `FOC_TICK_HZ` = 20 kHz, `PRG_FOC` has **50 µs per tick**. Everything else
must fit in what is left.

| Task | Period | Prio | Cost per scan | Notes |
|---|---|---|---|---|
| `SAFE` | 1 ms | 5 | ~1 µs, or ~25 µs every 5th | The ADC read is the spike |
| `FOC` | 50 µs | 4 | the bulk of the budget | `loopFOC()` + `move()` every 20th |
| `COMMS` | 1 ms | 3 | small, but `Serial.print` on gain changes is not | |
| `SER` | 100 ms | 2 | a whole telemetry line | Lowest priority, so it yields |

### The lesson that cost the most bench time

`PRG_SAFETY` runs **above** `PRG_FOC`. `io::vbus::readRaw()` is a *blocking* ADC
conversion of ~24 µs. Calling it every 1 ms steals ~24 µs from a 50 µs FOC
budget, at a perfectly fixed rate. Since notifications are not queued, the
stolen tick is lost rather than deferred.

The symptom was **a ~5 Hz velocity oscillation that appeared the moment
regenerative braking was added**. Not noise, not jitter — a periodic beat, from
a periodic theft.

The fix is the rate split in `PRG_SAFETY`: nFAULT at 1 kHz (a free digital
read), the chopper at 1 kHz (two register writes — this accelerates the *cut*,
not the regulation), and the ADC + protection ladder at `CFG_BUS_SAFETY_HZ`
(200 Hz, since nothing on a battery bus has 1 kHz dynamics).

> **Before you add work to `SAFE` or any task above `FOC`, ask what it costs per
> scan and how often it truly needs to run.** Blocking calls in a
> higher-priority task are the failure mode to watch for, and they show up as
> control-loop artefacts, not as an error.

---

## 8. Function blocks

### Calling convention

Every FB in this codebase follows the same shape: `operator()` takes the
`VAR_INPUT`s, advances the internal state by one scan, and returns the primary
`VAR_OUTPUT`. Secondary outputs are readable as members afterwards.

```cpp
if (fbOvTrip(vbus, gvl::M.fault)) { /* trip */ }
float duty = fbChopper(vbus, stage_active);
```

That reads like an ST network and keeps the program body about *sequence*, not
plumbing.

### Timebase: scans, not milliseconds

The timer blocks count **scans**. Every task here has a fixed period, so scans
are an exact and cheaper timebase: no `millis()` inside a 20 kHz loop, and no
49-day rollover to reason about. Declare presets readably:

```cpp
plc::TON _debounce{plc::scansFor(10, 1000 / CFG_BUS_SAFETY_HZ)};  // 10 ms
```

> **A block's preset is tied to the scan rate of the program that owns it.** If
> you move an FB instance to a task with a different period, its timings change
> silently. Re-derive the preset.

### The standard library

[`src/plc/plc_std_fb.h`](../src/plc/plc_std_fb.h), header-only. An unused block
generates no code, so the file costs nothing to keep complete.

| Block | Signature | Behaviour |
|---|---|---|
| `LIMIT` | `LIMIT(mn, in, mx)` | Clamp. `constexpr` function |
| `scansFor` | `scansFor(ms, scan_ms)` | Preset helper; rounds up, never returns 0 |
| `TON` | `TON{pt_scans}`, `(bool in)` | True once `in` held true for `pt` consecutive scans. Any false scan resets. Asserts **on** the `pt`-th scan (`>= pt`) |
| `R_TRIG` / `F_TRIG` | `(bool clk)` | Rising / falling edge, one scan wide |
| `RS` | `(bool set, bool reset)` | Reset-dominant latch |
| `RAMP_REAL` | `(target, max_step)` | Slew limiter. `max_step <= 0` = pass-through |
| `HYSTERESIS` | `(in, low, high)` | Rises when `in > high`, falls when `in < low`, holds between. Strict comparisons — the ones the chopper was commissioned with |

### The application blocks

[`src/fb/`](../src/fb/): `VbusFilter`, `OverVoltageTrip`, `BrakeChopper`,
`GateFault`, `regenDerate`, `regenClamp`, `velocityRamp`.

### When an FB should be a plain function

Two of the above are deliberately *functions*, not blocks, and the reasoning
generalises:

- **`regenDerate` / `regenClamp`** are stateless. In IEC terms a stateless POU
  is a FUNCTION. Wrapping them in a class would imply state that isn't there.

- **`velocityRamp`** *looks* stateful — it is a ramp — but its state **is**
  `gvl::Q.active_target`, which is process data: zeroed on disarm, and written
  directly by the position and torque modes. An FB holding its own copy would go
  stale across those transitions and, on re-arm, resume slewing from a setpoint
  nobody asked for. So it takes the current value as an input and returns the
  next one.

> **The general rule: state that other code resets or writes belongs in the
> process image, not inside a block.** Duplicating it is how the two copies
> diverge.

### Writing your own

```cpp
namespace fb {
class YourBlock {
 public:
  bool operator()(float in, bool enable) {   // VAR_INPUT
    /* one scan of work */
    return _q;                               // VAR_OUTPUT
  }
  float diagnostic() const { return _x; }    // secondary output
 private:
  plc::TON _delay{plc::scansFor(50, 1)};     // compose from the standard blocks
  bool  _q = false;
  float _x = 0.0f;
};
}
```

No globals, no hardware, no `Serial`. Those constraints are what let you test it
on a host compiler — and they are why *actions* (cutting EN_GATE, printing a
fault) stay in the program while the *decision* lives in the block.

---

## 9. The I/O layer

[`src/io/`](../src/io/) is the only code permitted to touch a register, a pin or
a peripheral. Everything else goes through it.

| Module | Owns | Key surface |
|---|---|---|
| `io_gate` | EN_GATE, nFAULT, DRV8301 SPI3 | `enable/disable/enabled/faultAsserted/setGain` |
| `io_motor` | TIM1 (6-PWM), TIM3/EXTI (sensor), current-sense ADC | the SimpleFOC object graph, `enableStage()`, `currentSenseAdc()` |
| `io_vbus` | one of ADC1/ADC2 | `init()`, `readRaw()` |
| `io_brake` | TIM2 CH3/CH4 | `preInit/init/setDuty/off/duty` |
| `io_can` | CAN1 (PB8/PB9) | the `OdriveCAN` instance |
| `io_console` | USB serial | `poll()`, the `ack*` helpers |

**Peripheral ownership is exclusive and worth keeping that way.** TIM1 motor,
TIM2 brake, TIM3 sensor, TIM6 FOC tick, and the two ADCs split between the
current sense and Vbus. That table is only checkable because the code is
confined to one directory.

### The output/logic split

`io_brake` is the cleanest example. It is **pure output**: give it a duty, it
programs the timer. The *decision* — hysteresis, proportional law — is
`fb::BrakeChopper`. So the register sequence with all the hard-won
center-aligned/dead-time/CCR-preload reasoning sits in one file that logic
changes never touch. Prefer this split for any new peripheral.

### The blocking exception: `src/seq/`

[`src/seq/`](../src/seq/) holds the commissioning sequences — hall angle
calibration (`H`) and R/L characterisation (`M`). They **block for seconds**
while driving the motor open-loop, which stalls the whole `COMMS` task.

That is acceptable only because all of the following hold, and any new sequence
must hold them too:

1. it runs with the motor **disarmed**;
2. it is triggered by an explicit operator request, never automatically;
3. `PRG_CONTROL` `return`s immediately after running one, giving up the rest of
   that scan;
4. `PRG_SAFETY` keeps running throughout — it is a higher-priority task, so bus
   protection is live even while a sequence blocks.

---

## 10. The three declaration tables

All three are X-macro lists in `include/`: the includer `#define`s the macro,
`#include`s the file, and `#undef`s it. Each is expanded more than once, which
is the entire point — one edit updates every derived artefact.

### `console_commands.h` → `src/prog/prog_console.cpp`

```c
CONSOLE_CMD(key, sub, group, help, handler)
```

Expanded twice: into the dispatch table, and into the boot banner. **Dispatch is
two passes** — exact `key`+`sub` first (two-letter commands, argument at
`line+2`), then `key` with `sub == 0` as a catch-all (single-letter commands,
argument at `line+1`). That is what makes `KP0.5` reach `cmdVelP` while both `K`
and `K5` reach `cmdVelReapply`, matching the original hand-written switch
exactly.

`help` of `""` hides a command from the banner — used for the I/D members of a
gain family whose P line already documents the set. The `group` column picks the
banner line.

### `can_commands.h` → `lib/odrive_can/`

```c
CAN_RX(cmd, handler)                    // void handler(const uint8_t* b)
CAN_TX_CYCLIC(cmd, period_ms, sender)   // void sender()
```

Expanded **four** times: the handler declarations in the header, a
`TX_CYCLIC_COUNT` counter, the RX dispatch switch, and the cyclic TX timers.
Both macros must be defined at every expansion site, even if to nothing.

A command may appear in both lists — the three telemetry getters answer an
explicit request *and* broadcast on a timer, as ODrive does. Declaring both here
is what stops those paths drifting.

Anything unlisted is silently ignored on RX, deliberately: an unknown frame from
a richer master must not fault us.

### `telemetry_schema.h` → firmware **and** web GUI

```c
TELEMETRY_CHANNEL(key, label, color, altkey, prec, expr)
TELEMETRY_CHANNEL_HALL(...)   // hall builds only
```

Compiled by both sides of the wire. The firmware uses `expr`; the GUI discards
it and uses `label`/`color`/`key`/`altkey`.

> ⚠️ **Its path is load-bearing.** `GUI/serial_plotter_wasm/src/channels.h`
> includes it *by bare name* through its CMake include path. Do not move it.
>
> `expr` must be a valid preprocessor argument: balanced parentheses, **no
> top-level commas**. It may reference firmware globals freely.

Adding a channel also means adding a demo value in the GUI's `demosource.cpp`.

---

## 11. Walkthroughs

### 11.1 A CAN velocity setpoint

```
CAN frame 0x00D
  → OdriveCAN::poll()            io/io_can, called by PRG_FIELDBUS_IN
  → rxSetInputVel(b)             rev/s → rad/s, feeds the watchdog
  → gvl::AXIS.input_vel
  → PRG_CONTROL::updateSetpoint()   same scan
      clamp to min(velocity_limit, CFG_VEL_CMD_MAX)
      fb::velocityRamp() at CFG_VEL_ACCEL
  → gvl::Q.active_target
  → PRG_FOC::scan()              next 50 µs tick
      motor.move(active_target)
      fb::regenClamp() on current_sp
  → TIM1 PWM
```

`CFG_VEL_CMD_MAX` is ~90 % of the no-load speed reachable under
`CFG_VOLT_LIMIT`. Past that the setpoint is physically unreachable: the PID
saturates and the integrator winds up without ever converging.

### 11.2 An over-voltage event

```
PRG_SAFETY, every 5th scan (200 Hz):
  io::vbus::readRaw()            blocking ~24 µs
  fb::VbusFilter                 median-of-3, THEN low-pass
  → gvl::IN.vbus_filt
  ├ fb::OverVoltageTrip          TON, 2 scans ≈ 10 ms
  │   → EN_GATE low, gvl::M.fault, ERR_DC_BUS_OVER_VOLTAGE, print
  └ fb::regenDerate → gvl::Q.regen_iq_limit → consumed by PRG_FOC

PRG_SAFETY, every scan (1 kHz):
  fb::BrakeChopper(vbus_filt, stage_active) → io::brake::setDuty()
```

Median **before** low-pass is not cosmetic: it rejects a single wild sample — an
aborted conversion, a commutation transient — *before* it enters the filter
state, where a low-pass would smear it across the following samples.

The debounce is not optional either: a transient must never latch the fault.

### 11.3 The CAN IRQ priority trap

`OdriveCAN::begin()` calls `setIRQPriority()` and `setAutoBusOffRecovery()`
**before** `_can.begin()`. Both are documented as setup-only.

Getting it wrong is silent. `preemptPriority` is never initialised by the
library's constructor, so calling `setIRQPriority()` *after* `begin()` left the
CAN1 ISR at NVIC priority 0 — **above** `configMAX_SYSCALL_INTERRUPT_PRIORITY`
— where it could preempt a FreeRTOS critical section mid-update inside the
driver's own ring buffers and corrupt them.

Symptom: *a few frames work, then everything wedges forever,
non-deterministically depending on boot timing.*

`begin()` also defaults `retransmission` to false despite its header comment,
and the constructor separately disables auto bus-off recovery — so a handful of
unacked frames at boot (before the peer's driver is up) could push the
controller into BUS_OFF, which then never recovers. Both are set explicitly.

---

## 12. The DC-bus safety ladder

Three stages, softest to hardest. **The threshold ordering in
[`motor_config.h`](../include/config/motor_config.h) is what makes it correct:**

```
BRAKE_VBUS_OFF < BRAKE_VBUS_ON < REGEN_START < REGEN_FULL < OV_TRIP
     24.2           24.6            26.5          27.5        29.0     (PSU)
```

| Stage | Block | Action |
|---|---|---|
| 1 | `fb::BrakeChopper` | Dissipate into the 2 Ω resistor, proportional duty above `BRAKE_VBUS_ON` |
| 2 | `fb::regenDerate` | Withdraw permission to brake electrically, linearly `REGEN_START → REGEN_FULL` |
| 3 | `fb::OverVoltageTrip` | Latched fault: EN_GATE low, motor freewheels |

Because the chopper's thresholds sit **below** the derate's, the resistor always
gets its chance before any braking torque is sacrificed. Stage 3 is only reached
if 1 and 2 both failed. **Preserve that ordering if you retune.**

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

## 13. Modifying core components

### Change a task's rate, priority or stack

Edit [`plc_config.h`](../include/config/plc_config.h). Before raising any
priority above `PRIO_FOC`, re-read [§7](#7-timing-budget). Before lowering a
stack, note that `configCHECK_FOR_STACK_OVERFLOW` is now live and will report a
task by name at runtime.

**Changing a task's period silently changes every FB preset inside it** — the
timer blocks count scans. Re-derive them.

### Add an event (interrupt-paced) task

1. Pick a free timer. TIM1/2/3/6 are taken (motor / brake / sensor / FOC tick).
2. Add the row: `plc::Task("NAME", plc::Event(TIM7, hz), prio, stack, PROGRAMS)`.
3. If it is the third event task, raise `PLC_MAX_EVENT_TASKS` in
   [`plc_runtime.h`](../src/plc/plc_runtime.h) **and** add a trampoline to the
   `TRAMPOLINE[]` array in `plc_runtime.cpp`. The boot check will tell you.

### Change the FOC rate

`FOC_TICK_HZ` in `plc_config.h`. Also check:

- `CFG_PWM_FREQ_HZ` — currently matched to it, which keeps the current-sense
  window sane;
- `MOTION_DOWNSAMPLE` — `move()` runs at `FOC_TICK_HZ / MOTION_DOWNSAMPLE`;
- `PrgFoc::TEL_DIV` — deliberately `FOC_TICK_HZ / 1000`, *not*
  `MOTION_DOWNSAMPLE`, so the telemetry rate does not silently follow a motion
  retune;
- the 50 µs budget in [§7](#7-timing-budget) scales inversely.

### Change the sensor

`SENSOR_TYPE` in `motor_config.h` selects hall vs quadrature at compile time.
The whole chain is built in `io_motor.cpp` and exposed as one `Sensor&
foc_sensor`, so **no program needs changing**. To add a third sensor type, add a
branch there and set `foc_sensor`.

The hall chain is three layers, each fixing the one below:
`HallSensorSmoothVel` (multi-edge velocity averaging + per-sector angle
correction) → `SmoothingSensor` (interpolates between 60° edges) → `HybridSensor`
(blends in the sensorless flux observer above `CFG_SENSORLESS_VEL_LO`).

### Replace or remove the fieldbus

`lib/odrive_can` only holds a reference to `gvl::AXIS`. Delete `io_can` and the
two comms programs, and the control logic is untouched. To add a *second*
fieldbus, write another I/O module writing the same block — but then check
[§6](#6-concurrency-and-memory-model): two writers to one variable breaks the
single-writer rule.

### Add a process-image variable

Add it to the right struct in `gvl.h`, name its single writer, keep it to one
32-bit word, mark it `volatile`. If two programs need to write it, restructure
so one owns it.

### Modify the runtime itself

`plc_runtime.cpp` is small on purpose. If you change a task body, preserve:
programs-before-delay, `vTaskDelayUntil` against a pre-loop baseline,
handle-published-before-timer-resume, and `NVIC_PRIO_RTOS_SAFE` on any ISR that
notifies a task. Each of those is load-bearing for reasons in [§4](#4-the-runtime-internals).

---

## 14. Compile-time configuration

| File | Holds | Change it when |
|---|---|---|
| [`config/hw_pinout.h`](../include/config/hw_pinout.h) | pins, peripheral assignment, board topology | The board changes |
| [`config/motor_config.h`](../include/config/motor_config.h) | motor, limits, bus thresholds, current-sense scaling, saved calibration, PID defaults | The motor or tuning changes |
| [`config/plc_config.h`](../include/config/plc_config.h) | task rates, priorities, stacks, NVIC policy | The schedule changes |
| [`STM32FreeRTOSConfig.h`](../include/STM32FreeRTOSConfig.h) | kernel configuration | Rarely — read its header first |

`include/board_config.h` is now a three-line umbrella including all three, kept
so the standalone bench sketches in `test/` still compile. **New code should
include the specific header it needs.**

Key switches: `SENSOR_TYPE` (hall/quadrature), `CFG_BUS_SOURCE` (PSU/battery —
changes thresholds only), `CFG_PRECALIBRATED`, `CFG_HALL_PRECALIBRATED`,
`CFG_SENSORLESS_ENABLE`, `CFG_WATCHDOG_MS` (0 = off; set ~250 on a vehicle so
losing the CAN master disarms).

### The build flag you must not remove

`platformio.ini` passes `-I$PROJECT_INCLUDE_DIR`. PlatformIO puts `include/` on
the path for `src/` but **not** for `lib/` targets, and two things need it:

- `lib/odrive_can` includes `gvl/axis_io.h`;
- the FreeRTOS library's `__has_include("STM32FreeRTOSConfig.h")` probe.

Without the flag that probe answers "no" and the kernel silently uses its own
defaults — which is how stack-overflow and malloc-failure detection sat switched
**off** for a long time despite the config file and hooks written to enable
them. The failure mode is silence. If those hooks stop firing, check this first.

---

## 15. Verifying a change

1. **`pio run`.** Compare flash/RAM against the previous build. A jump of
   kilobytes for a small change means an unintended copy or a lost `inline`.
2. **Diff the boot banner** against a known-good capture. It exercises DRV8301
   SPI, ADC allocation, current sense, brake thresholds and CAN in one shot.
3. **Serial smoke test**, motor free and PSU current-limited: `Q` (compare the
   `cfg` line field by field), `A`, `V5`, `V10`, `I`, `C`; then `M` and `H`
   disarmed.
4. **GUI**: if you touched `telemetry_schema.h`, confirm
   `GUI/serial_plotter_wasm/src/channels.h` still compiles against it and every
   channel still plots.
5. **CAN**, with `can_utilities`: heartbeat 10 Hz, encoder estimates 100 Hz,
   `Set_Axis_State(CLOSED_LOOP)` arms, `Set_Input_Vel` tracks.
6. **Bus safety**, if you touched `PRG_SAFETY` or `fb/`: spin up and brake — the
   chopper must engage at `BRAKE_VBUS_ON` and release at `BRAKE_VBUS_OFF`, the
   derate must start at `REGEN_START`, and the OV fault must latch at `OV_TRIP`
   with `err=0x4`.

---

## 16. Glossary

| IEC 61131-3 | Here | FreeRTOS equivalent |
|---|---|---|
| CONFIGURATION | `src/config/configuration.cpp` | — |
| RESOURCE | `src/plc/plc_runtime.cpp` | the scheduler |
| TASK | `plc::TaskDef` row in `TASKS[]` | one `xTaskCreate` |
| PROGRAM (POU) | `plc::Program` subclass in `src/prog/` | the task body |
| FUNCTION_BLOCK | class in `src/fb/` or `plc_std_fb.h` | — |
| FUNCTION | free `inline` function (stateless) | — |
| VAR_GLOBAL / process image | `gvl::IN` / `Q` / `M` / `AXIS` | shared globals |
| VAR_INPUT / VAR_OUTPUT | `operator()` arguments / return value | — |
| %I / %Q | `gvl::IN` / `gvl::Q`, via `src/io/` | — |
| Scan cycle | one pass of a task's program list | one loop iteration |
| SFC step sequence | `src/seq/` (blocking, operator-triggered) | — |

---

## See also

- [`src/README.md`](../src/README.md) — orientation and the "how do I add X" recipes
- [`Getting_Started.md`](Getting_Started.md) — flashing and driving the board
- [`Calibration.md`](Calibration.md) — commissioning a new motor
- [`GUI.md`](GUI.md) — the web plotter / PID tuner
