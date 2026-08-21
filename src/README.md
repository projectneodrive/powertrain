# Firmware architecture — orientation

Three layers, six modules, four tasks. The point of the structure is that adding
a feature should touch **one file plus one line**, and that the boundary between
*hardware access*, *control logic* and *scheduling* is visible in the layout
rather than asked for in a comment.

> 📖 **This page gets you productive.** For the internals — the runtime, the
> concurrency model, the timing budget, the safety ladder and how to change core
> components — see **[docs/Architecture.md](../docs/Architecture.md)**.

---

## Contents

- [The five-second version](#the-five-second-version)
- [Directory map](#directory-map)
- [Where does my code go?](#where-does-my-code-go)
- [How to add a feature](#how-to-add-a-feature)
- [Shared state, and the one rule that matters](#shared-state-and-the-one-rule-that-matters)
- [Reading an existing module](#reading-an-existing-module)
- [Conventions you are expected to follow](#conventions-you-are-expected-to-follow)
- [Things that look arbitrary but are not](#things-that-look-arbitrary-but-are-not)
- [Build notes](#build-notes)
- [Where to look when something breaks](#where-to-look-when-something-breaks)

---

## The five-second version

```
        src/boot.cpp — hardware bring-up order, and the four tasks
        ┌──────────────┬──────────────┬──────────────┬──────────────┐
        │  SAFE 1 kHz  │  FOC 20 kHz  │ COMMS 1 kHz  │  SER 10 Hz   │
        │  prio 5      │  prio 4      │  prio 3      │  prio 2      │
        └──────┬───────┴──────┬───────┴──────┬───────┴──────┬───────┘
               │              │              │              │
               ▼              ▼              ▼              ▼
            safety          foc         readFieldbus     console
                                        control
                                        publishTelemetry
               │              │              │              │
               └──────────────┴──────┬───────┴──────────────┘
                                     ▼
                              src/state.h
                    the values modules share, grouped by
                    WHO IS ALLOWED TO WRITE THEM
                                     │
                                     ▼
                              src/io/ — the ONLY code
                              that touches hardware
```

A task calls its modules in order, then waits for its next trigger. For `COMMS`
that order is the contract — read the fieldbus, run the logic, publish — so a CAN
setpoint arriving in a given millisecond is acted on and reported in that same
millisecond.

`main.cpp` is four lines of code. It calls `boot::run()`, which never returns.

---

## Directory map

| Path | What it is | Rule it obeys |
|---|---|---|
| [`boot.cpp`](boot.cpp) | Hardware bring-up order, the four task bodies, task creation | The only file that knows the whole system |
| [`app.h`](app.h) | Every module's entry points, in one file | Two lines per module |
| [`app/`](app/) | The control modules: `foc` `safety` `control` `comms` `console` `calibration` | One `.cpp` each. Owns its state as file-statics. Never creates a task, never touches a peripheral, never blocks* |
| [`io/`](io/) | Hardware access | The **only** code allowed to touch a register, a pin or a peripheral |
| [`state.h`](state.h) | The values modules share | Grouped by writer — see below |
| [`util/timers.h`](util/timers.h) | `Debounce`, `Hysteresis`, `limit` | Small, stateful, no hardware |
| [`config/`](config/) | `rtos_hooks.cpp` (stack-overflow / malloc-failure halt), `system_clock.cpp` | Linked by symbol name; no include edge |

\* `calibration` is the marked exception — see below.

Configuration constants are in [`include/config/`](../include/config/):

| File | Change it when |
|---|---|
| [`hw_pinout.h`](../include/config/hw_pinout.h) | the board changes |
| [`motor_config.h`](../include/config/motor_config.h) | the motor or the tuning changes |
| [`tasks_config.h`](../include/config/tasks_config.h) | the schedule changes |

---

## Where does my code go?

| What you're writing | Goes in | Why |
|---|---|---|
| Something that must run at a fixed rate | `app/` — a new module | A task in `boot.cpp` gives it a rate and a priority |
| Reusable logic with memory between calls (a timer, a filter, a latch) | `util/` if two modules need it; a file-static in the module if only one does | Seven single-consumer helper headers used to live in their own directory — they were folded back into their one caller |
| Anything touching a register, pin or peripheral | `io/` | Peripheral ownership stays analysable |
| A value two modules must share | [`state.h`](state.h), in the struct named for its writer | See below |
| A one-shot operator procedure that blocks | `app/calibration.cpp` | And read the four conditions in [Architecture §7.6](../docs/Architecture.md#76-calibration--blocking-on-request) |
| A new serial / CAN / telemetry entry | the matching table in [`include/`](../include/) | One line, and every derived artefact updates — including the GUI and the ESP32 station |

---

## How to add a feature

### A new cyclic behaviour → a new module

**1.** Write `app/yourthing.cpp`:

```cpp
#include "app.h"

#include "io/io_motor.h"
#include "state.h"

namespace app {
namespace yourthing {
namespace {

uint16_t s_counter = 0;      // module state: file-static, never a global

}  // namespace

void update() {
  if (state::safety.vbus_filt < 20.0f) {
    state::control.active_target = 0.0f;   // only if you OWN that struct
  }
}

}  // namespace yourthing
}  // namespace app
```

**2.** Declare it in [`app.h`](app.h) — two lines, with the rate it expects:

```cpp
// 1 kHz. What it does, and anything a caller must know.
namespace yourthing { void update(); }
```

**3.** Call it from a task in [`boot.cpp`](boot.cpp):

```cpp
void commsTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    app::comms::readFieldbus();
    app::control::update();
    app::yourthing::update();      // <- here
    app::comms::publishTelemetry();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SCAN_MS_COMMS));
  }
}
```

**Position in that sequence is a design decision, not a formality.** Before
`control::update()` means you influence this scan's control decision; after it
means you observe the result. Before `publishTelemetry()` means your effect is
published this scan; after means next scan.

### A new task

A `Spec` row in `createTasks()` plus a body next to the others in `boot.cpp`.
Take the priority and stack from `tasks_config.h` rather than hard-coding them.

> ⚠️ Before giving anything a priority above `PRIO_FOC`, read the
> [timing budget](../docs/Architecture.md#6-timing-budget). A blocking call in a
> higher-priority task shows up as a control-loop artefact, not as an error.

### A new serial command

One line in [`include/console_commands.h`](../include/console_commands.h) plus a
handler in `app/console.cpp`:

```c
//          key  sub  group      help                     handler
CONSOLE_CMD('Z',  0,  GRP_CMDS,  "Z<v> your thing",       cmdYourThing)
```

```cpp
void cmdYourThing(float v) {
  float old = state::axis.something;
  state::axis.something = v;
  io::console::ackFloat("Z", "something", old, v, 2, "unit");
}
```

The boot banner is generated from the same table, so the help cannot drift from
the commands that exist. `help` of `""` hides an entry from the banner.

> ⚠️ [`can_utilities`](../can_utilities/README.md) builds **its** console from
> this same table, so it will not compile until it has a handler too — mapping
> the command onto CANSimple, or stating that it cannot be. Answer it there and
> move on. The handler *names* are frozen by that shared expansion: you may move
> them, not rename them.

### A new CAN command

One line in [`include/can_commands.h`](../include/can_commands.h) plus a handler
in `lib/odrive_can/odrive_can.cpp`:

```c
CAN_RX(CMD_YOUR_THING, rxYourThing)                 // void(const uint8_t* b)
CAN_TX_CYCLIC(CMD_YOUR_THING, 100, sendYourThing)   // void(), every 100 ms
```

That table drives RX dispatch, the cyclic TX timers **and** the handler
declarations in the header. The arbitration ids are in
[`can_ids.h`](../include/can_ids.h).

> ⚠️ A new `CAN_TX_CYCLIC` line fails `can_utilities`' link until it has a
> decoder — a new broadcast cannot be silently dropped by the control station.

### A new telemetry channel

One line in [`include/telemetry_schema.h`](../include/telemetry_schema.h). The
firmware streams it and the web GUI plots it. Also give it a demo value in the
GUI's `demosource.cpp`.

> ⚠️ **Its path is load-bearing**: the GUI includes it by bare name through its
> CMake include path. Do not move it. The `expr` column must be a valid
> preprocessor argument — balanced parentheses, **no top-level commas**.

⚠️ `can_utilities` also compiles it and needs one accessor per channel, so a new
channel fails its link until somebody decides whether that value is reachable
over CAN at all. Several are not, and say so explicitly.

### A new axis error bit

One line in [`include/axis_vocab.h`](../include/axis_vocab.h), which carries the
value **and** the operator-facing name:

```c
AXIS_ERROR(ERR_YOUR_THING, 0x00001000, "YOUR_THING")
```

That generates the enum in [`axis_io.h`](../include/axis_io.h), the decoder
`can_utilities` uses to log `error 0x0 -> 0x1000 [YOUR_THING]`, and the web GUI's
CAN Devices page. Raise it with `state::axis.raiseError(ERR_YOUR_THING)` — never
with `|=`, see below.

### A new shared variable

Add it to the [`state.h`](state.h) struct named for its writer, keep it to one
32-bit word, mark it `volatile`. If no struct fits, the value probably has two
writers — resolve that first.

---

## Shared state, and the one rule that matters

Everything shared between modules lives in [`state.h`](state.h) and nowhere else,
**grouped by who writes it**:

| Struct | Writer | Contents |
|---|---|---|
| `state::foc` | `app::foc` | shaft angle/velocity, Iq, electrical power |
| `state::safety` | `app::safety` | filtered Vbus, regen limit, the fault latch |
| `state::control` | `app::control` | motion setpoint, `foc_ready`, `calibrated` |
| `state::at_boot` | boot only | `isense_ok` |
| `state::req` | set by anyone, consumed by one | request flags |
| `state::axis` | **multi-writer by design** | the CAN-mapped command/telemetry block |

**There are no mutexes, no queues and no critical sections between modules.**
That is safe only because of one rule:

> **Every shared variable has exactly one writer, and the struct it lives in is
> named after that writer.**

On Cortex-M4 an aligned 32-bit load or store is atomic, so a single-writer
`float` can be read from any task without synchronisation. **Pairs of variables
are not**, and that is the concrete bug this rule exists to prevent:
`Sensor::getAngle()` reads `(full_rotations, angle_prev)`, a pair updated at
20 kHz. Reading it from another task returned torn values — **±1 turn spikes in
`pos_rev`**. Hence the design you see: `app::foc` is the only code that touches
the sensor, and it publishes independent floats everyone else reads.

The same argument produced `state::foc.p_elec` (four dq terms, one published
product) and put the clear-errors handling inside `app::safety` (one owner for
the fault latch).

| Do | Don't |
|---|---|
| Put it in the struct named for its writer | Write the same variable from two modules |
| Keep it to a single 32-bit word | Share a struct, array or 64-bit value unprotected |
| Mark it `volatile` | Rely on `volatile` for atomicity — it only prevents register caching |
| Derive multi-field state in one place, publish the result | Publish two fields a reader must combine |
| `state::axis.raiseError(BIT)` | `state::axis.axis_error \|= BIT` — a lost-update race across two priorities |

Everything firmware-side is **SI/rad**. Only the CAN boundary converts to rev/Nm.

Full variable-by-variable detail:
[Architecture §4](../docs/Architecture.md#4-shared-state).

---

## Reading an existing module

Good ones to read first, in order:

1. **[`app/comms.cpp`](app/comms.cpp)** — 29 lines of code, shows the module
   shape and nothing else.
2. **[`boot.cpp`](boot.cpp)** — the whole system in one file: bring-up order,
   four tasks, what each runs.
3. **[`app/safety.cpp`](app/safety.cpp)** — the DC-bus ladder end to end, and the
   clearest example of *why* rates differ within one task.
4. **[`app/control.cpp`](app/control.cpp)** — the axis state machine; the largest
   module, and the one whose ordering matters most.

---

## Conventions you are expected to follow

| Convention | Rationale |
|---|---|
| One module per `.cpp`, entry points declared in `app.h` | The declaration is the registration |
| Module state is file-static in an anonymous namespace, never a global | Scopes it, and keeps `state.h` for things that are genuinely shared |
| Modules never block, never `delay()`, never create tasks | A delay inside a scan silently re-rates everything after it |
| Hardware only in `src/io/` | Keeps peripheral ownership checkable in one place |
| `req_*` flags: **clear first, unconditionally, then act — and always report** | Clearing only on success is how a command gets stuck looking dead |
| Comments in English; **operator-facing serial strings left byte-identical** | Host tooling parses them |
| `util::Debounce` takes its period as an argument | A preset in "calls" is meaningless without the rate that calls it |

---

## Things that look arbitrary but are not

Each of these cost bench time to find. They are load-bearing.

- **The `SAFE` task splits its rates.** nFAULT is polled at the full 1 kHz; the
  Vbus ADC runs at 200 Hz. The conversion blocks for ~24 µs, and this task
  outranks `foc` — at 1 kHz it stole a FOC tick often enough to produce a visible
  **~5 Hz speed oscillation**. RTOS notifications are not queued, so a missed
  tick is lost, not caught up.
- **The brake chopper is updated at 1 kHz anyway.** That is not accelerating the
  regulation, it is accelerating the *cutoff*: a disarm takes effect in 1 ms
  instead of 5, for two register writes.
- **The brake cannot work with the stage disarmed.** The LM5109B's VDD comes from
  GVDD, which only exists while `EN_GATE` is high. This is board wiring, not
  policy — and it means there is *no* over-voltage protection while disarmed.
- **Vbus is on its own ADC.** Sharing the current sense's ADC lets TIM1's
  injected trigger abort a regular conversion mid sample-and-hold; it resumes
  with the injected channel's voltage still in the S/H. That produced a
  systematic false OV fault during alignment.
- **The CAN IRQ priority is set *before* `begin()`.** Setting it after leaves the
  ISR at NVIC priority 0, above `configMAX_SYSCALL_INTERRUPT_PRIORITY`, where it
  can preempt a FreeRTOS critical section and corrupt the driver's own ring
  buffers. Symptom: a few frames work, then everything wedges forever.
- **`app::foc` never writes `motor.shaft_angle`/`shaft_velocity`.** `move()`
  stores the filtered multi-turn angle there; overwriting mixes two reference
  frames into the telemetry.
- **The velocity ramp is a plain function.** Its state *is*
  `state::control.active_target`, which is zeroed on disarm and written directly
  by the position and torque modes. A helper holding its own copy would go stale
  and resume slewing from a setpoint nobody asked for.
- **The DC-bus threshold ordering** (`BRAKE_OFF < BRAKE_ON < REGEN_START <
  REGEN_FULL < OV_TRIP`) is what guarantees the resistor always gets its chance
  before braking torque is sacrificed, and before the fault. Preserve it if you
  retune.
- **The FOC task publishes its handle before starting TIM6.** The ISR fires as
  soon as the timer runs; publishing afterwards leaves a window where the first
  notifications are dropped — intermittent, boot-only, and it looks like nothing.
- **The OV debounce has a `static_assert` on it.** Lowering `CFG_BUS_SAFETY_HZ`
  erodes the debounce silently, and at 100 Hz it would vanish entirely: the
  last-resort fault would latch on a single ADC sample.

---

## Build notes

`platformio.ini` passes `-I$PROJECT_INCLUDE_DIR`. This is not cosmetic:
PlatformIO puts `include/` on the compiler path for `src/` but **not** for
`lib/` targets, and two things depend on it —

- `lib/odrive_can` includes `axis_io.h`, the axis block it is mapped onto;
- the FreeRTOS library's `__has_include("STM32FreeRTOSConfig.h")` probe. Without
  the flag it answers "no" and the kernel silently uses its own defaults instead
  of `include/STM32FreeRTOSConfig.h` — including leaving stack-overflow and
  malloc-failure detection switched **off**.

The second one failed silently for a long time. If those hooks ever stop firing,
check that flag first.

**`include/` is a published interface.** Three hardcoded relative paths point
into it (`can_utilities/platformio.ini`, the GUI's `CMakeLists.txt` and its
`tests/CMakeLists.txt`), and the GUI includes two headers by bare name. Moving it
breaks two other projects, neither of which is built by CI here.

---

## Where to look when something breaks

| Symptom | Look at |
|---|---|
| A task halts with a named stack-overflow message | `STACK_*` in [`tasks_config.h`](../include/config/tasks_config.h) — the check is live and names the task |
| Periodic oscillation at a task's rate | Something blocking in a task above `foc` — [Architecture §6](../docs/Architecture.md#6-timing-budget) |
| A few CAN frames then a permanent wedge | IRQ priority / bus-off recovery — [§10.3](../docs/Architecture.md#103-the-can-irq-priority-trap) |
| `±1` turn jumps in position | A non-atomic pair read across tasks — [§5](../docs/Architecture.md#5-concurrency-and-memory-model) |
| A serial command does nothing | A `req_*` flag cleared only on the success path |
| Hard hang on the first FOC tick | The TIM6 NVIC priority — [§3.2](../docs/Architecture.md#32-the-foc-event-task) |
| False over-voltage during alignment | ADC sharing — [`hw_pinout.h`](../include/config/hw_pinout.h) near `PIN_VBUS` |
| Boot stops partway | Diff the banner against a known-good capture; each line marks a bring-up step |

---

## See also

- **[docs/Architecture.md](../docs/Architecture.md)** — the full reference:
  boot, the tasks, shared state, concurrency, timing, the modules, the safety
  ladder, walkthroughs, and how to modify core components
- [docs/Getting_Started.md](../docs/Getting_Started.md) — flashing and driving the board
- [docs/Calibration.md](../docs/Calibration.md) — commissioning a new motor
- [docs/GUI.md](../docs/GUI.md) — the web plotter / PID tuner / CAN devices page
