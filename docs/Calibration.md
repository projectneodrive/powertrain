# Commissioning a new motor — calibration procedure

This is the step‑by‑step for bringing up a **new motor**: which parameters to
find, how to measure them, and how to **save** them so the board boots ready.

> **How "saving" works today:** parameters live in
> [`include/config/motor_config.h`](../include/config/motor_config.h) and are baked in at
> compile time (the ODrive‑style `pre_calibrated` equivalent). You measure once,
> paste the numbers into that file, and rebuild. Runtime save‑over‑CAN‑to‑flash
> is a later phase; until then, **`config/motor_config.h` + rebuild is the save.**

## The parameters

| # | Parameter | How to get it | `motor_config.h` field |
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

In [`config/motor_config.h`](../include/config/motor_config.h), set the values directly, and
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

Copy them into `config/motor_config.h` — **note the µH → H conversion** (`250 µH = 0.00025`):

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

Copy both straight into `config/motor_config.h`:

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

## Step 7 — the counter‑torque brake *(CTRL_BRAKE, optional)*

`B<Nm>` puts the axis in `CTRL_BRAKE`: it applies a braking torque of that
magnitude whose **sign always opposes rotation**. It never drives — stop turning
the shaft and the torque goes to zero.

> This replaced a constant-**power** mode (`W<watts>`, `Iq = -P/(Kt·ω)`). That law
> diverged as ω → 0: it saturated at the current limit across most of the usable
> range, and because its sign came from the sign of ω it flipped on hall
> quantisation noise — motoring the shaft in alternating directions and collapsing
> the bus from 24 V to 6 V. Constant torque removes the divergence. **It does not
> remove the sign problem**, which is why every threshold below still matters.

**Prerequisites:** current sense active (the board refuses to arm in this mode
without it). `CFG_GEAR_RATIO` affects only the cadence telemetry, not the brake.

### The speed window

Three numbers bound the mode, all in **motor rad/s**, all printed at boot on the
`[brake]` line and by `Q`:

| | Comes from | On the motor fitted today |
|---|---|---|
| `CFG_REGEN_OMEGA_FLOOR` | the sensor: 4 hall edges inside `CFG_HALL_VEL_WINDOW` | 3.22 rad/s |
| `CFG_REGEN_OMEGA_ON` / `_OFF` | your choice, clear of the floor | 4.5 / 3.4 rad/s |
| `CFG_BEMF_OMEGA_MAX` | the motor and the pack | 20.2 rad/s |

The **floor** is the one that is not negotiable. The braking current takes its
sign from the sign of ω, so below the speed at which the velocity estimate
resolves its own sign that current is a full-magnitude command pointing in a
random direction. `src/app/safety.cpp` static_asserts `OMEGA_ON` against it, and
the law refuses below it regardless of configuration.

The **ceiling** is where the line back-EMF reaches the bus: past it the
free-wheel diodes rectify into the bus and the inverter has no authority at all
— disarming makes it worse, not better, because the gates were what held it off.

These thresholds are deliberately **not** derived from `CFG_GEAR_RATIO`. Making
them scale with it once turned a bench run at `G = 1` into a 72× threshold change
and the oscillation described above. To set them from a target cadence, write
`CFG_REGEN_CRANK_RPM(15.0f)`, which evaluates where you write it.

### What the setpoint buys you

Unlike the power law, the torque is delivered **exactly** and the envelope is no
longer speed-dependent: any value up to `CFG_REGEN_T_MAX` holds at any speed in
the window. That ceiling is the current limit expressed as a torque
(`CFG_REGEN_I_MAX · Kt` ≈ 4.03 Nm), and the law clamps against the **live** limit,
so lowering `LC` lowers the accepted torque with it.

What does rise with speed is the **power**, `P = T·ω`. At full setpoint near the
top of the window that is ~80 W into the resistor — close to
`CFG_BRAKE_P_CONT` = 100 W, so this is the mode where the thermal derate actually
shows up. Setpoint changes are slew-limited by `CFG_REGEN_T_SLEW` (8 Nm/s, ~0.5 s
to full) so a step does not jolt the shaft or spike the bus.

### Bench procedure

1. `Q` — check `regen_t_max`, `regen_t_slew`, `regen_on`, `regen_off`,
   `regen_floor`, `bemf_omega_max`. Compare against the boot `[brake]` line.
2. `A` with the shaft at rest: `Iq` must read `0.00`, and **the shaft must turn
   freely by hand**. That is the "no movement without an external force"
   requirement, and it is the floor delivering it.
3. `V12` to spin up, then `B1.0`. `Iq` should settle at ≈ −0.99 A and **stay
   there** as the speed decays — a flat line, not the ±4 A square wave the power
   law produced. `Iq` must always be opposite in sign to `vel`.
4. `B4`, then `B0`: the change should take ~0.5 s, not one sample. Then `I` and
   `A` again while the shaft still turns — the torque must restart from zero and
   ramp, not snap back to the old value.
5. Let it coast down. Between ~6.5 and 4.5 rad/s the torque should withdraw
   **gradually** (that is `CFG_REGEN_RAMP_BAND`), then be exactly zero below
   `CFG_REGEN_OMEGA_OFF`, with no chatter around the threshold.
6. `B0` and `B-2` must both give zero torque.
7. **Energy balance.** `Pres` should track `B · ω_motor` — a check against a
   number you chose, which is why no mechanical-power channel is needed. On a lab
   supply `Pbat` should sit near zero: the supply cannot sink, so the chopper has
   to be taking everything. A `Pbat` that stays positive means `CFG_BRAKE_R` is
   wrong or the AUX half-bridge is not conducting — `Vbus` arbitrates.
8. **Thermal derate.** Hold `B4` at ~18 rad/s (≈ 80 W) for a minute or two:
   `Irgn` must fall away smoothly as the model withdraws braking current, with no
   fault and no step the shaft can feel. It recovers over
   `CFG_BRAKE_THERMAL_TAU_S`.

> The resistor thermal model is the one thing here no voltage reading can check
> for you. In PSU mode the chopper holds `Vbus` flat while burning power
> continuously, so the bus looks perfectly healthy right up until the resistor
> fails. Set `CFG_BRAKE_P_CONT` from the part's datasheet, not from the peak
> rating `CFG_BRAKE_P_MAX_W`.

**Known limitation.** The dead zone is `CFG_REGEN_OMEGA_ON` = 4.5 rad/s, i.e.
43 rpm at the motor in direct drive — noticeable if you turn the shaft slowly by
hand. Geared at `G = 72` that is 0.6 rpm of cadence, so it is negligible on the
pedal rig. Going lower would mean deriving direction from the **hall sector
transitions** (resolved on the first edge, ~0.04 rad) rather than from the
velocity estimate.

---

## Quick reference — one full example (`config/motor_config.h`)

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
  yet; edit `config/motor_config.h` and reflash. (Runtime flash persistence is planned.)
- If `initFOC` fails (`ENCODER_FAILED`), the sensor isn't reading correctly —
  check wiring/`SENSOR_TYPE`, and for hall verify the A/B/C order on PB4/PB5/PC9.
- `Kt` is only as good as the datasheet KV; if torque readings are off, refine
  `CFG_KV`.
- **Re‑run the hall calibration** (Step 3b, `H`) whenever you remount the hall
  PCB, change the magnet ring, or swap the motor — the offsets are tied to the
  physical hall‑to‑magnet alignment. Rewiring the phases or halls also invalidates
  it (re‑do Step 3 **and** 3b).
