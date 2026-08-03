# Firmware architecture — orientation

This firmware is organised as an **IEC 61131-3 style PLC**. If you have written
ladder or structured text on a real PLC, the vocabulary here means what you
expect it to mean. If you haven't, [§16 of the reference](../docs/Architecture.md#16-glossary)
maps every term onto plain C++ and FreeRTOS.

The point of the structure is that adding, removing or changing a feature should
touch **one file plus one line**, and that the boundary between *hardware
access*, *control logic* and *scheduling* is enforced by the layout rather than
by comments asking you not to cross it.

> 📖 **This page gets you productive.** For the internals — the runtime, the
> concurrency model, the timing budget, and how to change core components —
> see **[docs/Architecture.md](../docs/Architecture.md)**.

---

## Contents

- [The five-second version](#the-five-second-version)
- [Directory map](#directory-map)
- [Where does my code go?](#where-does-my-code-go)
- [How to add a feature](#how-to-add-a-feature)
- [The process image, and the one rule that matters](#the-process-image-and-the-one-rule-that-matters)
- [Reading an existing program](#reading-an-existing-program)
- [Conventions you are expected to follow](#conventions-you-are-expected-to-follow)
- [Things that look arbitrary but are not](#things-that-look-arbitrary-but-are-not)
- [Build notes](#build-notes)
- [Where to look when something breaks](#where-to-look-when-something-breaks)

---

## The five-second version

```
                  src/config/configuration.cpp
                  ┌──────────────────────────────────────────┐
                  │  CONFIGURATION: the TASKS[] table        │
                  │  + the hardware boot order               │
                  └───────────────┬──────────────────────────┘
                                  │ declares
        ┌─────────────┬───────────┴───────────┬──────────────┐
        ▼             ▼                       ▼              ▼
   TASK "SAFE"   TASK "FOC"             TASK "COMMS"    TASK "SER"
   cyclic 1 ms   event 20 kHz (TIM6)    cyclic 1 ms     cyclic 100 ms
   prio 5        prio 4                 prio 3          prio 2
        │             │                       │              │
        ▼             ▼                       ▼              ▼
   PRG_SAFETY    PRG_FOC            PRG_FIELDBUS_IN     PRG_CONSOLE
                                    PRG_CONTROL
                                    PRG_TELEMETRY_OUT
        │             │                       │              │
        └─────────────┴───────────┬───────────┴──────────────┘
                                  ▼
                        gvl:: — the process image
                     (the ONLY channel between programs)
                                  │
                                  ▼
                        src/io/ — the ONLY code
                        that touches hardware
```

A task fires, runs each of its programs **once, in the order declared**, then
waits for its next trigger. For `COMMS` that order *is* the classic PLC scan —
read inputs, run logic, write outputs — so a CAN setpoint arriving in a given
millisecond is acted on and reported in that same millisecond.

`main.cpp` is 31 lines. It calls `configuration::boot()` and never returns.

---

## Directory map

| Directory | IEC concept | Rule it obeys |
|---|---|---|
| [`config/`](config/) | CONFIGURATION | Declares the task table and the boot order. The only file that knows the whole system. |
| [`plc/`](plc/) | RESOURCE + standard library | The runtime: `Program`, `TaskDef`, the FreeRTOS binding, and the standard function blocks (`TON`, `R_TRIG`, `RS`, `RAMP_REAL`, `HYSTERESIS`, `LIMIT`). Knows nothing about motors. |
| [`prog/`](prog/) | PROGRAM (POU) | Cyclic logic. Owns function block instances as members. Never creates a task, never touches a peripheral, never blocks. |
| [`fb/`](fb/) | FUNCTION_BLOCK | Reusable stateful logic with explicit inputs/outputs. No globals, no hardware — which is what makes them testable without a board. |
| [`io/`](io/) | I/O modules | The **only** code allowed to touch a register, a pin or a peripheral. |
| [`seq/`](seq/) | — | Blocking commissioning sequences (hall calibration, R/L characterisation). The deliberate exception to "programs never block". |
| [`gvl/`](gvl/) | VAR_GLOBAL | Definition of the process image. Types live in [`include/gvl/`](../include/gvl/). |

Configuration constants are in [`include/config/`](../include/config/), split
three ways:

| File | Change it when |
|---|---|
| [`hw_pinout.h`](../include/config/hw_pinout.h) | the board changes |
| [`motor_config.h`](../include/config/motor_config.h) | the motor or the tuning changes |
| [`plc_config.h`](../include/config/plc_config.h) | the schedule changes |

`include/board_config.h` is now a three-line umbrella over those, kept so the
standalone bench sketches in `test/` still compile. **New code should include the
specific header it needs.**

---

## Where does my code go?

| What you're writing | Goes in | Why |
|---|---|---|
| Something that must run at a fixed rate | `prog/` — a new PROGRAM | The task table gives it a rate and a priority |
| Reusable logic with memory between scans (a timer, a filter, a latch) | `fb/` — a FUNCTION_BLOCK | Keeps state scoped and the logic testable |
| Reusable logic with **no** memory | `fb/` — a plain `inline` function | A stateless POU is a FUNCTION, not a block; a class would imply state that isn't there |
| Anything touching a register, pin or peripheral | `io/` — an I/O module | Peripheral ownership stays analysable |
| A value two programs must share | `include/gvl/gvl.h` | And name its single writer — see below |
| A one-shot operator procedure that blocks | `seq/` | And read the four conditions in [§9](../docs/Architecture.md#the-blocking-exception-srcseq) |
| A new serial / CAN / telemetry entry | the matching table in `include/` | One line, and every derived artefact updates |

---

## How to add a feature

### A new cyclic behaviour → a new PROGRAM

**1.** Write `prog/prog_yourthing.{h,cpp}`, deriving from `plc::Program`:

```cpp
// prog_yourthing.h
#pragma once
#include "plc/plc_program.h"
#include "fb/fb_something.h"

namespace prog {

class PrgYourThing : public plc::Program {
 public:
  const char* name() const override { return "PRG_YOURTHING"; }
  void init() override;   // optional: runs once, before the scheduler
  void scan() override;   // runs once per task cycle

 private:
  fb::SomeBlock _fbSomething;   // FB instances are members, never globals
  uint16_t      _counter = 0;
};

extern PrgYourThing prgYourThing;

}  // namespace prog
```

```cpp
// prog_yourthing.cpp
#include "prog/prog_yourthing.h"
#include "gvl/gvl.h"

namespace prog {

PrgYourThing prgYourThing;          // the single instance

void PrgYourThing::init() { /* seed defaults; Serial is available here */ }

void PrgYourThing::scan() {
  if (_fbSomething(gvl::IN.vbus_filt)) {
    gvl::Q.active_target = 0.0f;    // read/write the process image
  }
}

}  // namespace prog
```

**2.** Register it — add it to a task's program list in
[`config/configuration.cpp`](config/configuration.cpp). That is the whole
registration:

```cpp
plc::Program* const COMMS_PROGRAMS[] = {
  &prog::prgFieldbusIn,
  &prog::prgControl,
  &prog::prgYourThing,      // <- here
  &prog::prgTelemetryOut,
};
```

**Position in that list is a design decision, not a formality.** Before
`PRG_CONTROL` means you influence this scan's control decision; after it means
you observe the result. Before `PRG_TELEMETRY_OUT` means your effect is
published this scan; after means next scan.

### A new task

Add one row to `TASKS[]`:

```cpp
plc::Task("NAME", plc::Cyclic(10),        PRIO_X, STACK_X, YOUR_PROGRAMS),
plc::Task("NAME", plc::Event(TIM7, 5000), PRIO_X, STACK_X, YOUR_PROGRAMS),
```

Take the priority and stack from [`plc_config.h`](../include/config/plc_config.h)
rather than hard-coding numbers. Event tasks need a slot in
`PLC_MAX_EVENT_TASKS` (`plc/plc_runtime.h`) — the runtime halts loudly at boot
if the table asks for more than exist.

> ⚠️ Before giving anything a priority above `PRIO_FOC`, read the
> [timing budget](../docs/Architecture.md#7-timing-budget). A blocking call in a
> higher-priority task shows up as a control-loop artefact, not as an error.

### A new function block

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

The convention: `operator()` takes the inputs, advances one scan, returns the
primary output; secondary outputs are members. No globals, no hardware, no
`Serial` — those constraints are what let you test it on a host compiler, and
they are why *actions* (cutting EN_GATE, printing a fault) stay in the program
while the *decision* lives in the block.

> ⏱️ The timer blocks count **scans, not milliseconds**. A block's preset is
> tied to the period of the program that owns it — move the instance to a
> different task and its timings change silently. Declare presets with
> `plc::scansFor(ms, scan_ms)` so the source still reads in real time units.

### A new serial command

One line in [`include/console_commands.h`](../include/console_commands.h) plus a
handler in `prog/prog_console.cpp`:

```c
//          key  sub  group      help                     handler
CONSOLE_CMD('Z',  0,  GRP_CMDS,  "Z<v> your thing",       cmdYourThing)
CONSOLE_CMD('Z', 'X', GRP_CONFIG,"ZX<v> sub-command",     cmdYourThingX)
```

```cpp
void cmdYourThing(float v) {
  float old = gvl::AXIS.something;
  gvl::AXIS.something = v;
  io::console::ackFloat("Z", "something", old, v, 2, "unit");
}
```

The boot banner is generated from the same table, so the help can no longer
drift from reality — which is exactly what it did when both were hand-written.
`help` of `""` hides an entry from the banner.

⚠️ [`can_utilities`](../can_utilities/README.md) builds **its** console from this
same table, so it will not compile until it has a handler too — mapping the
command onto CANSimple, or saying it cannot be. That is the point; answer it
there and move on.

### A new CAN command

One line in [`include/can_commands.h`](../include/can_commands.h) plus a handler
in `lib/odrive_can/odrive_can.cpp`:

```c
CAN_RX(CMD_YOUR_THING, rxYourThing)                       // void(const uint8_t* b)
CAN_TX_CYCLIC(CMD_YOUR_THING, 100, sendYourThing)         // void(), every 100 ms
```

That one table drives RX dispatch, the cyclic TX timers **and** the handler
declarations in the header, so all three stay in step. The arbitration ids
themselves are in [`include/can_ids.h`](../include/can_ids.h), split out so a
host that cannot compile `STM32_CAN.h` still shares them.

⚠️ A new `CAN_TX_CYCLIC` line fails `can_utilities`' link until it has a decoder
for the frame — a new broadcast cannot be silently dropped by the control
station.

### A new telemetry channel

One line in [`include/telemetry_schema.h`](../include/telemetry_schema.h). The
firmware streams it and the web GUI plots it — that file is compiled by both
sides. Also give it a demo value in the GUI's `demosource.cpp`.

> ⚠️ **Its path is load-bearing**: `GUI/serial_plotter_wasm` includes it by bare
> name through its CMake include path. Do not move it. The `expr` column must be
> a valid preprocessor argument — balanced parentheses, **no top-level commas**.

⚠️ `can_utilities` also compiles it, and needs one accessor per channel, so a new
channel fails its link until somebody decides whether that value is reachable
over CAN at all. Several are not, and say so explicitly.

### A new axis error bit (or state, or control mode)

One line in [`include/axis_vocab.h`](../include/axis_vocab.h), which carries the
value **and** the operator-facing name:

```c
AXIS_ERROR(ERR_YOUR_THING, 0x00001000, "YOUR_THING")
```

That single line generates the enum in [`gvl/axis_io.h`](../include/gvl/axis_io.h),
the decoder `can_utilities` uses to log `error 0x0 -> 0x1000 [YOUR_THING]`, and
the web GUI's CAN Devices page. Before it existed, each of those three carried
its own copy of the names, so a new bit appeared as undecoded hex in two of them.

`ERR_NONE` is deliberately not in that list — it is the absence of bits, and a
decoder walking the table would match it against every value.

### A new shared variable

Add it to the right struct in [`include/gvl/gvl.h`](../include/gvl/gvl.h),
**name its single writer in a comment**, keep it to one 32-bit word, mark it
`volatile`. Read the next section before you do.

---

## The process image, and the one rule that matters

Everything shared between programs lives in `gvl::` and nowhere else:

| Area | Struct | Contents |
|---|---|---|
| `%I` | `gvl::IN` | measured values — shaft angle/velocity, filtered Vbus, current-sense status |
| `%Q` | `gvl::Q` | commands produced by the logic — motion setpoint, regen current limit |
| `%M` | `gvl::M` | machine state — fault, foc_ready, calibrated |
| — | `gvl::AXIS` | the fieldbus-mapped command/telemetry block |

**There are no mutexes, no queues and no critical sections between programs.**
That is safe only because of one rule:

> **Every process-image variable has exactly one writer, and it is named in a
> comment beside it.**

On Cortex-M4 an aligned 32-bit load or store is atomic, so a single-writer
`float` can be read from any task without synchronisation. **Pairs of variables
are not**, and that is the concrete bug this rule exists to prevent:
`Sensor::getAngle()` reads `(full_rotations, angle_prev)`, a pair updated at
20 kHz. Reading it from another task returned torn values — **±1 turn spikes in
`pos_rev`**. Hence the design you see: `PRG_FOC` is the only code that touches
the sensor, and publishes two independent floats everyone else reads.

| Do | Don't |
|---|---|
| One writer, named in a comment | Write the same variable from two programs |
| Keep it to a single 32-bit word | Share a struct, array, or 64-bit value unprotected |
| Mark it `volatile` | Rely on `volatile` for atomicity — it only prevents register caching |
| Derive multi-field state in one place, publish the result | Publish two fields a reader must combine |

Everything firmware-side is **SI/rad**. Only the CAN boundary converts to rev/Nm.

Full variable-by-variable table, including writers and readers:
[Architecture §5](../docs/Architecture.md#5-the-process-image).

---

## Reading an existing program

Good ones to read first, in order:

1. **[`prog/prog_comms.h`](prog/prog_comms.h)** — 35 lines, shows the scan shape.
2. **[`prog/prog_safety.cpp`](prog/prog_safety.cpp)** — a program composing four
   function blocks, and the clearest example of *why* rates differ within one
   task.
3. **[`config/configuration.cpp`](config/configuration.cpp)** — the whole system
   in one file.
4. **[`prog/prog_control.cpp`](prog/prog_control.cpp)** — the axis state machine;
   the largest program, and the one whose scan order matters most.

---

## Conventions you are expected to follow

| Convention | Rationale |
|---|---|
| One PROGRAM per file pair, one instance, `extern` in the header | Instances are referenced by the task table |
| FB instances are program members, prefixed `_fb` | Scopes state; makes two instances possible |
| Programs never block, never `delay()`, never create tasks | A delay inside a scan silently re-rates everything after it |
| Hardware only in `src/io/` | Keeps peripheral ownership checkable in one place |
| `req_*` flags: **clear first, unconditionally, then act — and always report** | Clearing only on success is how a command gets stuck looking dead |
| Comments in English; **operator-facing serial strings left byte-identical** | Host tooling may parse them |
| Timer presets via `plc::scansFor(...)` | Keeps real time units visible in a scan-counted timebase |

---

## Things that look arbitrary but are not

Each of these cost bench time to find. They are load-bearing.

- **The `SAFE` task splits its rates.** nFAULT is polled at the full 1 kHz; the
  Vbus ADC runs at 200 Hz. The conversion blocks for ~24 µs, and this task
  outranks `PRG_FOC` — at 1 kHz it stole a FOC tick often enough to produce a
  visible **~5 Hz speed oscillation**. RTOS notifications are not queued, so a
  missed tick is lost, not caught up.
- **The brake chopper is updated at 1 kHz anyway.** That is not accelerating the
  regulation, it is accelerating the *cutoff*: a disarm takes effect in 1 ms
  instead of 5, for two register writes.
- **The brake cannot work with the stage disarmed.** The LM5109B's VDD comes
  from GVDD, which only exists while `EN_GATE` is high. This is board wiring, not
  policy — and it means there is *no* over-voltage protection while disarmed.
- **Vbus is on its own ADC.** Sharing the current sense's ADC lets TIM1's
  injected trigger abort a regular conversion mid sample-and-hold; it resumes
  with the injected channel's voltage still in the S/H. That produced a
  systematic false OV fault during alignment.
- **The CAN IRQ priority is set *before* `begin()`.** Setting it after leaves the
  ISR at NVIC priority 0, above `configMAX_SYSCALL_INTERRUPT_PRIORITY`, where it
  can preempt a FreeRTOS critical section and corrupt the driver's own ring
  buffers. Symptom: a few frames work, then everything wedges forever.
- **`PRG_FOC` never writes `motor.shaft_angle`/`shaft_velocity`.** `move()`
  stores the filtered multi-turn angle there; overwriting mixes two reference
  frames into the telemetry.
- **The velocity ramp is a function, not a function block.** Its state *is*
  `gvl::Q.active_target`, which is zeroed on disarm and written directly by the
  position and torque modes. A block holding its own copy would go stale and
  resume slewing from a setpoint nobody asked for.
- **The DC-bus threshold ordering** (`BRAKE_OFF < BRAKE_ON < REGEN_START <
  REGEN_FULL < OV_TRIP`) is what guarantees the resistor always gets its chance
  before braking torque is sacrificed, and before the fault. Preserve it if you
  retune.

---

## Build notes

`platformio.ini` passes `-I$PROJECT_INCLUDE_DIR`. This is not cosmetic:
PlatformIO puts `include/` on the compiler path for `src/` but **not** for
`lib/` targets, and two things depend on it —

- `lib/odrive_can` includes `gvl/axis_io.h`, the process-image block it is
  mapped onto;
- the FreeRTOS library's `__has_include("STM32FreeRTOSConfig.h")` probe. Without
  the flag it answers "no" and the kernel silently uses its own defaults instead
  of `include/STM32FreeRTOSConfig.h` — including leaving stack-overflow and
  malloc-failure detection switched **off**.

The second one failed silently for a long time. If those hooks ever stop firing,
check that flag first.

---

## Where to look when something breaks

| Symptom | Look at |
|---|---|
| A task halts with a named stack-overflow message | `STACK_*` in [`plc_config.h`](../include/config/plc_config.h) — the check is live and names the task |
| Periodic oscillation at a task's rate | Something blocking in a task above `PRG_FOC` — [Architecture §7](../docs/Architecture.md#7-timing-budget) |
| A few CAN frames then a permanent wedge | IRQ priority / bus-off recovery — [Architecture §11.3](../docs/Architecture.md#113-the-can-irq-priority-trap) |
| `±1` turn jumps in position | A non-atomic pair read across tasks — [Architecture §6](../docs/Architecture.md#6-concurrency-and-memory-model) |
| A serial command does nothing | A `req_*` flag cleared only on the success path |
| False over-voltage during alignment | ADC sharing — [`hw_pinout.h`](../include/config/hw_pinout.h) near `PIN_VBUS` |
| Boot stops partway | Diff the banner against a known-good capture; each line marks a bring-up step |

---

## See also

- **[docs/Architecture.md](../docs/Architecture.md)** — the full reference:
  runtime internals, boot order, process image, concurrency, timing, the safety
  ladder, and how to modify core components
- [docs/Getting_Started.md](../docs/Getting_Started.md) — flashing and driving the board
- [docs/Calibration.md](../docs/Calibration.md) — commissioning a new motor
- [docs/GUI.md](../docs/GUI.md) — the web plotter / PID tuner
