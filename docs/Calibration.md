# Commissioning a new motor — calibration procedure

This is the step‑by‑step for bringing up a **new motor**: which parameters to
find, how to measure them, and how to **save** them so the board boots ready.

> **How "saving" works today:** parameters live in
> [`include/board_config.h`](../include/board_config.h) and are baked in at
> compile time (the ODrive‑style `pre_calibrated` equivalent). You measure once,
> paste the numbers into that file, and rebuild. Runtime save‑over‑CAN‑to‑flash
> is a later phase; until then, **board_config.h + rebuild is the save.**

## The parameters

| # | Parameter | How to get it | `board_config.h` field |
|---|-----------|---------------|------------------------|
| 1 | Pole pairs | Datasheet, or (magnets ÷ 2) | `CFG_POLE_PAIRS` (via `MOTOR_PRESET`) |
| 2 | KV → torque const `Kt` | Datasheet (`Kt = 8.27/KV`) | `CFG_KV` |
| 3 | Sensor type | Your hardware | `SENSOR_TYPE` |
| 4 | Encoder PPR (quadrature only) | Encoder datasheet | `CFG_ENC_PPR` |
| 5 | Phase resistance `R` | **Measure** (`M` command) | `CFG_PHASE_R` |
| 6 | Phase inductance `L` | **Measure** (`M` command) | `CFG_PHASE_L` |
| 7 | Sensor electrical offset | **Measure** (arm once) | `CFG_ZERO_ELEC_ANGLE` |
| 8 | Sensor direction | **Measure** (arm once) | `CFG_SENSOR_DIRECTION` |
| 9 | Current / velocity / voltage limits | Your design | `CFG_CURRENT_LIMIT`, `CFG_VEL_LIMIT`, `CFG_VOLT_LIMIT` |
| 10 | Current‑loop PID | Tune | `CFG_CUR_P`, `CFG_CUR_I` |
| 11 | Velocity‑loop PID | Tune (live: serial `KP`/`KI`/`KD` or CAN `Set_Vel_Gains`) | `CFG_VEL_P`, `CFG_VEL_I` |

Items 1–4 come from the **datasheet**; 5–8 are **measured by the board**; 9–11
are your **design choices / tuning**.

---

## Step 0 — safety

Motor **free to spin**, PSU **current‑limited** (1–2 A for first tests), hand on
the power switch. Calibration energises and moves the motor.

## Step 1 — set the known parameters (datasheet)

In [`board_config.h`](../include/board_config.h), pick a preset or set the values
directly, and **leave `CFG_PRECALIBRATED 0`** for now:

```c
#define MOTOR_PRESET  MOTOR_PRESET_EBIKE   // or _BENCH, or set the fields by hand:
// #define CFG_POLE_PAIRS 26
// #define CFG_KV         8.2f
#define SENSOR_TYPE   SENSOR_TYPE_HALL     // or SENSOR_TYPE_QUADRATURE
// #define CFG_ENC_PPR 600                 // quadrature only
#define CFG_PRECALIBRATED 0
```
Build + flash + open the serial monitor (115200). You should see
`DRV8301 ... gain_set=OK` and `Current sense OK -> foc_current`.

## Step 2 — measure phase resistance & inductance (`R`, `L`)

With the motor **free**, type `M` in the serial monitor (or send CAN
`Set_Axis_State(4)` — MOTOR_CALIBRATION). The board energises briefly and prints:

```
Characterising motor (R/L)...
  R = 0.1234 ohm   L = 250.00 uH
```

Copy them into `board_config.h` — **note the µH → H conversion** (`250 µH = 0.00025`):

```c
#define CFG_PHASE_R  0.1234f      // ohm  (as printed)
#define CFG_PHASE_L  0.00025f     // H    (printed µH ÷ 1e6)
```

## Step 3 — find the sensor offset & direction

Type `A` to arm (or CAN `Set_Axis_State(8)`). The motor **twitches** as it
aligns the sensor to the electrical angle, then prints:

```
initFOC OK | CFG_SENSOR_DIRECTION=1  CFG_ZERO_ELEC_ANGLE=2.7183
```

Copy both straight into `board_config.h`:

```c
#define CFG_ZERO_ELEC_ANGLE  2.7183f
#define CFG_SENSOR_DIRECTION 1        // +1 = CW, -1 = CCW (as printed)
```

## Step 4 — save & use it (pre‑calibrated boot)

Now flip the switch on and rebuild:

```c
#define CFG_PRECALIBRATED 1
```
Build + flash. The board now applies the saved offset/direction and **skips both
the sensor‑alignment sweep and the current‑sense verification** — the rotor no
longer has to rotate freely to arm, so you can arm with the wheel on the ground.

> The current‑sense check (`skip_align`) follows `CFG_PRECALIBRATED`: with `0`
> the first arm verifies shunt polarity/pin order (and prints `CS: Inv B`,
> `CS: Switch B-C`… if it had to correct something — if so, fix the wiring or
> gains so a pre‑calibrated boot matches reality); with `1` it trusts the
> configuration as‑is. Re‑run steps 2–3 only if you change the motor, remount
> the sensor, or rewire the phases/halls.

## Step 5 — tune the control loops

- **Current loop** (`CFG_CUR_P`, `CFG_CUR_I`): command a small torque (`T0.5`),
  watch `Iq=` track the setpoint smoothly (no buzz/oscillation). Raise `P` for
  faster response; back off if it whines.
- **Velocity loop**: tune **live** over serial — `KP0.3`, `KI2`, `KD0` set the
  velocity PID on the fly (`K` alone prints the applied gains; each change is
  echoed as `[PID vel] P=… I=… D=…`). The same works over CAN with
  `Set_Vel_Gains` (0x01B, ODrive units). Command a speed (`V5`), tune for a
  firm, non‑oscillating response, then **save the result** into `CFG_VEL_P` /
  `CFG_VEL_I` (units: A/(rad/s) — the echoed values) — gains are not persisted
  across reboots.
- **Limits**: set `CFG_CURRENT_LIMIT` (A) for your motor/battery. Keep
  `CFG_VOLT_LIMIT` conservative for bring‑up — **raise it toward Vbus** once the
  current loop is trusted, otherwise the motor can't reach speed (BEMF eats the
  voltage budget). `CFG_VEL_LIMIT` caps top speed, and velocity commands are
  additionally clamped to `CFG_VEL_CMD_MAX` (~90 % of the no‑load speed under
  `CFG_VOLT_LIMIT`) since asking for an unreachable speed only winds up the PID.

---

## Quick reference — one full example (`board_config.h`)

```c
#define MOTOR_PRESET         MOTOR_PRESET_EBIKE   // 26 pp, KV 8.2
#define SENSOR_TYPE          SENSOR_TYPE_HALL
#define CFG_CURRENT_LIMIT    15.0f
#define CFG_VOLT_LIMIT       24.0f                // raised after bring-up
// --- measured & saved ---
#define CFG_PHASE_R          0.1234f
#define CFG_PHASE_L          0.00025f
#define CFG_ZERO_ELEC_ANGLE  2.7183f
#define CFG_SENSOR_DIRECTION 1
#define CFG_PRECALIBRATED    1
```

## Notes & limits

- **Compile‑time persistence only.** There is no over‑CAN "save_configuration"
  yet; edit `board_config.h` and reflash. (Runtime flash persistence is planned.)
- If `initFOC` fails (`ENCODER_FAILED`), the sensor isn't reading correctly —
  check wiring/`SENSOR_TYPE`, and for hall verify the A/B/C order on PB4/PB5/PC9.
- `Kt` is only as good as the datasheet KV; if torque readings are off, refine
  `CFG_KV`.
