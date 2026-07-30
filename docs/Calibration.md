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
| 1 | Pole pairs | Datasheet, or (magnets ÷ 2) | `CFG_POLE_PAIRS` |
| 2 | KV → torque const `Kt` | Datasheet (`Kt = 8.27/KV`) | `CFG_KV` |
| 3 | Sensor type | Your hardware | `SENSOR_TYPE` |
| 4 | Encoder PPR (quadrature only) | Encoder datasheet | `CFG_ENC_PPR` |
| 5 | Phase resistance `R` | **Measure** (`M` command) | `CFG_PHASE_R` |
| 6 | Phase inductance `L` | **Measure** (`M` command) | `CFG_PHASE_L` |
| 7 | Sensor electrical offset | **Measure** (arm once) | `CFG_ZERO_ELEC_ANGLE` |
| 8 | Sensor direction | **Measure** (arm once) | `CFG_SENSOR_DIRECTION` |
| 9 | Hall sector angles (hall only) | **Measure** (`H` command) | `CFG_HALL_CAL_OFFSETS` |
| 10 | Current / velocity / voltage limits | Your design | `CFG_CURRENT_LIMIT`, `CFG_VEL_LIMIT`, `CFG_VOLT_LIMIT` |
| 11 | Current‑loop PID | Tune | `CFG_CUR_P`, `CFG_CUR_I` |
| 12 | Velocity‑loop PID | Tune (live: serial `KP`/`KI`/`KD` or CAN `Set_Vel_Gains`) | `CFG_VEL_P`, `CFG_VEL_I` |

Items 1–4 come from the **datasheet**; 5–9 are **measured by the board**; 10–12
are your **design choices / tuning**. Item 9 is **hall‑specific** and only
matters if you care about low‑ripple smoothness — skip it for a quadrature
encoder, which is smooth by construction.

---

## Step 0 — safety

Motor **free to spin**, PSU **current‑limited** (1–2 A for first tests), hand on
the power switch. Calibration energises and moves the motor.

## Step 1 — set the known parameters (datasheet)

In [`board_config.h`](../include/board_config.h), set the values directly, and
**leave `CFG_PRECALIBRATED 0`** for now:

```c
#define CFG_POLE_PAIRS 26
#define CFG_KV         8.2f
#define SENSOR_TYPE    SENSOR_TYPE_HALL   // or SENSOR_TYPE_QUADRATURE
// #define CFG_ENC_PPR 600                // quadrature only
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

## Step 3b — (hall only) calibrate the hall sector angles

**Skip this for a quadrature encoder.** It only helps hall‑sensored motors.

SimpleFOC assumes each of the 6 hall sectors is exactly 60° electrical. Real
hall/magnet placement is a few degrees off, *differently per sector*, so the FOC
commutation angle is wrong inside each sector → **torque ripple at every speed**
(felt as roughness that no velocity‑PID tuning removes). Symptom check: command a
constant torque (`T0.3`) and watch `Iq=`; if it *ripples* while the setpoint is
constant — and the **disarmed** shaft turns without strong detents — the roughness
is commutation error, not the velocity loop or magnetic cogging.

With the motor **free to spin** and **disarmed** (`I` first), type `H`:

```
AK H: hall-angle calibration requested (moteur désarmé)
Hall cal: spin boucle ouverte ~10s (moteur doit tourner lentement)...
Hall cal OK (dir=1). CFG_HALL_CAL_OFFSETS =
{
   0.0012345f,  // secteur 0  (1.84 deg elec)
  -0.0008765f,  // secteur 1  (-1.31 deg elec)
   ...
};
```

The board spins the motor **slowly in open loop for ~10 s** (this is expected —
don't touch it): it drives a rotating field at a known electrical angle, so the
*commanded* angle is the *true* angle, and it measures how far each hall sector's
real transition sits from the 60° grid. The 6 printed values are **mean‑zero**
mechanical corrections (the global offset stays with `CFG_ZERO_ELEC_ANGLE`), and
they're **applied immediately in RAM** for the rest of the session.

Then **re‑arm** (`A`) so `initFOC` recomputes `CFG_ZERO_ELEC_ANGLE` /
`CFG_SENSOR_DIRECTION` *on top of* the corrected hall angles — use **those**
re‑armed values in Step 3 above. Test `V2` / `V5`: the `Iq` ripple at constant
speed should drop noticeably.

Save the 6 numbers and enable them at boot:

```c
#define CFG_HALL_PRECALIBRATED 1
#define CFG_HALL_CAL_OFFSETS  { 0.0012345f, -0.0008765f, /* …4 more… */ }
```

> **Tuning the spin:** if `H` prints `secteur peu/non vu` the rotor didn't turn —
> raise `CFG_HALL_CAL_VOLTAGE` (3 V → 4–5 V). `CFG_HALL_CAL_ELEC_SPEED` /
> `CFG_HALL_CAL_REVS` control how slow/long the sweep is. `H` blocks the comms
> loop for the duration (like `M`), so no CAN traffic while it runs.
>
> **Expectation:** this removes most of the commutation ripple, but hall FOC (only
> 156 angle points/rev) is never as smooth as an encoder. What remains after a
> good calibration is the hall resolution itself.
>
> **A once‑per‑rev "click" is not a hall‑placement problem.** The 6 offsets are
> averaged over all 26 electrical cycles, so a *single* mislocated transition (1
> of the 156 per rev) would be washed out. We tried a per‑transition (156‑bin)
> calibration to catch exactly that: it found **no outlier** — the worst
> transition matched its sector's normal offset — and applying 156 individually
> measured corrections just injected measurement noise, making motion *worse*. So
> a regular click that survives the 6‑sector calibration is **magnetic**, not a
> commutation‑angle error: a slightly stronger/weaker magnet gives a once‑per‑rev
> torque/BEMF variation that hall‑angle calibration cannot correct (it fixes
> *timing*, not *torque*). Confirm it's not mechanical first (disarm, turn the
> shaft by hand — the shaft should feel smooth). The real cure for a torque‑based
> ripple is a quadrature encoder **with index** (absolute, repeatable position →
> enables true position‑indexed anti‑cogging); the hall has no absolute zero.

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

## Step 6 — sensorless above the crossover *(hall only, optional)*

The hall's 156‑states/rev quantization is the smoothness floor above ~5 rad/s
(Step 3b). A **flux observer** (MESC/Lemming, `src/HybridSensor.h`) estimates the
angle from the phase currents + applied voltages (BEMF integration) with **no
quantization**, so handing commutation over to it at speed removes that floor.
`HybridSensor` runs both sensors every FOC tick and blends hall → observer over
`[CFG_SENSORLESS_VEL_LO, CFG_SENSORLESS_VEL_HI]` (default 5→7 rad/s); below the
band it is bit‑for‑bit the hall‑only build.

**Prerequisites:** current sense active (not the voltage fallback) and
`CFG_PHASE_R` / `CFG_PHASE_L` measured (Step 1, command `M`). The observer's flux
linkage is derived from `CFG_KV` / `CFG_POLE_PAIRS`.

> **The observer must be VERIFIED before you trust it.** A wrong observer angle at
> speed is a mis‑commutation → violent motion / overcurrent. Bring‑up is staged so
> the handoff is *off* while you check the estimate:

1. Leave `CFG_SENSORLESS_ENABLE 0`, build + flash. The observer runs in the
   background (no effect on commutation) and two fields appear in telemetry:
   `obsdV=` (observer velocity − hall velocity, rad/s) and `blnd=` (observer
   fraction, 0…1).
2. Arm, sweep the speed across and above the crossover (`V4`, `V6`, `V8`, `V10`).
   Watch **`obsdV`**: it must stay **near 0** (a few tenths) at every speed once
   moving. `blnd` stays `0.00` (handoff disabled). If `obsdV` is large, wrong‑sign,
   or grows with speed → the observer is unhealthy (recheck `R`/`L`/`KV`, current
   sense) — **do not enable**.
3. Once `obsdV` is consistently small, set `CFG_SENSORLESS_ENABLE 1`, rebuild.
   Now `blnd` ramps 0→1 across `[VEL_LO, VEL_HI]`; above `VEL_HI` the observer
   drives commutation. The handoff has two safety gates: it never blends toward
   the observer until its **direction is latched** *and* its **velocity agrees**
   with the hall (`|obsdV|` within tolerance) — otherwise it stays on the hall.

Tune the band with `CFG_SENSORLESS_VEL_LO` / `_VEL_HI` (keep `VEL_LO` high enough
that the BEMF is strong — well above stall; a wider band = smoother handoff). No
zero/offset to save: the observer is slaved to the hall's calibrated
`CFG_ZERO_ELEC_ANGLE` at runtime, so the same commutation reference is used
through the handoff.

---

## Quick reference — one full example (`board_config.h`)

```c
#define CFG_POLE_PAIRS       26
#define CFG_KV               8.2f
#define SENSOR_TYPE          SENSOR_TYPE_HALL
#define CFG_CURRENT_LIMIT    15.0f
#define CFG_VOLT_LIMIT       24.0f                // raised after bring-up
// --- measured & saved ---
#define CFG_PHASE_R          0.1234f
#define CFG_PHASE_L          0.00025f
#define CFG_ZERO_ELEC_ANGLE  2.7183f
#define CFG_SENSOR_DIRECTION 1
#define CFG_PRECALIBRATED    1
// --- hall smoothness (hall only; Step 3b) ---
#define CFG_HALL_PRECALIBRATED 1
#define CFG_HALL_CAL_OFFSETS  { 0.0012f, -0.0009f, 0.0004f, 0.0007f, -0.0011f, -0.0003f }
// --- sensorless above the crossover (hall only; Step 6) ---
#define CFG_SENSORLESS_ENABLE  0                  // 1 after verifying obsdV~0
#define CFG_SENSORLESS_VEL_LO  5.0f
#define CFG_SENSORLESS_VEL_HI  7.0f
```

## Notes & limits

- **Compile‑time persistence only.** There is no over‑CAN "save_configuration"
  yet; edit `board_config.h` and reflash. (Runtime flash persistence is planned.)
- If `initFOC` fails (`ENCODER_FAILED`), the sensor isn't reading correctly —
  check wiring/`SENSOR_TYPE`, and for hall verify the A/B/C order on PB4/PB5/PC9.
- `Kt` is only as good as the datasheet KV; if torque readings are off, refine
  `CFG_KV`.
- **Re‑run the hall calibration** (Step 3b, `H`) whenever you remount the hall
  PCB, change the magnet ring, or swap the motor — the offsets are tied to the
  physical hall‑to‑magnet alignment. Rewiring the phases or halls also invalidates
  it (re‑do Step 3 **and** 3b).
