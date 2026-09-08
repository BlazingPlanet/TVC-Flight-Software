# Control Gain Tuning

**Project:** `06_stmport`
**Date measured:** 2026-09-02
**Vehicle:** flight configuration, inert motor at loaded F15 mass
**Motor:** Estes F15-0

Servo geometry, slew rate, and pin assignments live in `CALIBRATION.md`. This file covers
the plant model and the control gains derived from it.

---

## Measured vehicle properties

| Quantity | Symbol | Value |
|---|---|---|
| Mass, flight configuration | m | 0.701 kg |
| CG from aft end | — | 488 mm |
| Gimbal pivot to CG (lever arm) | L_arm | 429 mm |
| Pitch moment of inertia about CG | I | **0.0668 kg·m²** |

---

## Moment of inertia measurement

### Method

Bifilar (two-string) suspension, strings vertical and parallel at rest, attachment points
symmetric about the CG. Two modes were excited separately:

- **Swing mode** — rocket translates without rotating. Period depends only on string
  length, so this validates the setup and timing without depending on the vehicle at all.
- **Pitch mode** — rocket rotates in the vertical plane while the two-string geometry holds
  the CG fixed. This carries the pitch inertia.

Holding the CG fixed is what makes this preferable to a single-point hang: `I_cg` comes out
directly, with no parallel-axis correction and no need to measure a pivot-to-CG distance
precisely.

Small amplitude (under ~10°), secondary motion allowed to damp before timing, 10
oscillations counted and divided.

### Raw measurements

| Quantity | Symbol | Value |
|---|---|---|
| String length, attachment to rocket | L | 2.275 m |
| Attachment point separation | b | 0.560 m |
| Swing mode period | T_swing | 3.048 s |
| Pitch mode period | T_pitch | 3.336 s |

### Setup validation (swing mode)

```
T_swing = 2π·√(L/g)   →   L_implied = g·T² / (4π²)
                        = 9.807 × 3.048² / 39.478
                        = 2.308 m
```

Measured 2.275 m, implied 2.308 m — **1.4% agreement.** The small excess is expected, since
the effective pendulum length runs to the system CG rather than to the attachment points.
Setup and timing method confirmed sound.

### Pitch inertia

```
I_cg = m·g·(b/2)²·T_pitch² / (4π²·L)
     = 0.701 × 9.807 × 0.28² × 3.336² / (4π² × 2.275)
     = 0.0668 kg·m²
```

**Sanity check.** A uniform slender rod about its center is `m·ℓ²/12` ≈ 0.058 kg·m² for a
~1 m body. The measured value sits 15% above that, consistent with mass concentrated toward
the ends (motor aft, electronics and recovery forward). Order of magnitude confirmed.

---

## Plant model

A rocket rotates about its CG. Nozzle deflection δ produces a torque, and angular
acceleration is that torque over the pitch inertia:

```
τ = T · L_arm · sin(δ) ≈ T · L_arm · δ        (small angle, δ in rad)

θ̈ = (T · L_arm / I) · δ
```

The coefficient `K = T·L_arm/I` is the entire plant — the gain from commanded deflection to
angular acceleration.

At F15 average thrust (~15 N, from 49.6 N·s over 3.45 s):

```
K = 15 × 0.429 / 0.0668 = 96 s⁻² per radian
```

So 6° of gimbal produces roughly 10 rad/s² of angular acceleration. Substantial authority.

### Plant variation across the burn

**`K` is not constant.** The F15 thrust curve peaks near 25 N shortly after ignition and
tails off through the burn, so `K` swings roughly **±40%** around the average. Propellant
burn-off also shifts the CG aft-to-forward, changing `L_arm` slightly.

With fixed gains, the loop is effectively faster early in the burn and slower late. This is
normal for amateur TVC and is the primary argument against designing at the edge of the
stability envelope.

---

## Gain derivation

With PD control the closed loop is second order:

```
θ̈ + K·Kd·θ̇ + K·Kp·θ = 0
```

Matching to the standard form `θ̈ + 2ζω_n·θ̇ + ω_n²·θ = 0`:

```
Kp = ω_n² / K
Kd = 2·ζ·ω_n / K
```

These are dimensionless (rad of deflection per rad of error). The firmware's `Kp_att` and
`Kd_att` are in **gimbal degrees per radian**, so both are multiplied by 57.3.

### Design parameters

`ω_n` and `ζ` are **chosen, not measured** — they specify desired closed-loop behavior. The
choice of `ω_n` is bounded above by actuator bandwidth (see `CALIBRATION.md`): servo
transit gives ~20 Hz, and the standard 5× separation between loop and actuator puts the
ceiling near 4 Hz / 25 rad/s. A further haircut accounts for the 20 ms PWM frame latency.

`ζ = 0.7` throughout — the conventional choice, fast with mild overshoot.

### Candidate gains

| ω_n | Kp_att | Kd_att | Settling | Saturates at tilt |
|---|---|---|---|---|
| 19 rad/s | 215 | 15.8 | 300 ms | 1.6° |
| 15 rad/s | 134 | 12.5 | 380 ms | 2.6° |
| **12 rad/s** | **86** | **10** | **475 ms** | **4.0°** |
| 10 rad/s | 60 | 8.3 | 570 ms | 5.8° |

The saturation column is the deciding factor. `MAX_DEFLECT` is 6°, so a proportional gain
of 215 commands full deflection at only 1.6° of tilt — beyond that the controller is
saturated and behaving as bang-bang rather than PD. The ζ = 0.7 damping prediction is then
meaningless in exactly the regime where it matters.

Saturation is not inherently wrong; full authority is desirable when badly off-vertical.
But a loop that spends most of its time saturated is not the loop the gain math describes.

### Selected for first flight

```c
const float Kp_att = 86.0f;   // gimbal degrees per radian of attitude error
const float Kd_att = 10.0f;   // gimbal degrees per rad/s of body rate
```

**ω_n = 12 rad/s (1.9 Hz), ζ = 0.7.**

Rationale:

- Linear behavior maintained out to 4° of tilt, which covers the expected disturbance range
- ~475 ms settling, roughly 7 correction cycles across the 3.45 s burn — adequate authority
- Leaves margin against the two effects not measured: thrust-induced servo slowdown, and
  the ±40% plant variation across the burn
- Conservative in the direction that matters. A sluggish rocket that flies imperfectly
  produces a log to tune from; an oscillating one produces wreckage.

Gains can be raised for flight two with real log data. The reverse trade is worse.

---

## Confidence and error sources

The derivation is sound but the inputs carry real uncertainty. Ranked by impact on `Kp`:

| Source | Effect on Kp | Notes |
|---|---|---|
| Thrust variation across burn | ±40% | Unavoidable with fixed gains |
| `b` measurement | 2× the % error in b | `(b/2)²` term — measure attachment to attachment |
| Thrust-load servo slowdown | Unquantified | Bench measured with inertial load only |
| Period timing | ~2× the % error | 10-oscillation averaging keeps this small |
| Lever arm, CG shift in burn | ~10% | CG moves forward as propellant depletes |

`Kp` scales with `ω_n²`, so any error in the actuator bandwidth ceiling propagates squared.
This is the main reason for choosing ω_n well below the computed limit rather than at it.

**Attitude estimate quality is a separate and equally important input.** Perfect gains on a
drifting estimate produce confident, well-damped corrections toward the wrong vertical. See
the boost-phase drift section below — measured at ~0.45° accumulated by burnout, which
consumes roughly 11% of the linear range before saturation.

---

## Validation plan

The first flight is the validation. Nothing here has been confirmed in flight.

**From the decoded log, check:**

- `cmd_y`/`cmd_z` against `pulse_y`/`pulse_z` — does the slew limiter bind? Constant
  binding through boost means gains are too aggressive for the actuator.
- `LOG_FLAG_SAT_Y` / `LOG_FLAG_SAT_Z` — how often is the controller saturated? Occasional
  is fine; continuous means `Kp` is too high or the vehicle is more disturbed than expected.
- Attitude response shape — overshoot and ring-out indicate `Kd` too low or `Kp` too high.
  Sluggish return to vertical with no overshoot indicates room to raise `ω_n`.
- Timing between `err_y` appearing and the attitude responding — reveals actuator lag beyond
  what was modeled.

---

## Boost-phase estimator drift

**Measured 2026-09-03**, three runs, vehicle stationary and near-vertical on a solid
surface.

### Why this needs measuring

The Mahony filter fuses gyro and accelerometer. The gyro integrates rotation but drifts;
the accelerometer measures gravity's direction and bounds that drift. The `trust` term
weights the accelerometer by how close its magnitude is to 1 g.

During the burn, acceleration is roughly 2 g, so total measured acceleration is ~2.2 g.
`g_err` sits well past `TRUST_ZERO` (0.30), and **`trust` goes to exactly zero for the
entire burn.** This is correct behavior — under thrust, the accelerometer's "down" points
backward along the body axis, and correcting toward it would actively corrupt attitude.

The consequence is that for all 3.45 s of powered flight, attitude runs on **open-loop gyro
integration** with nothing bounding the drift. That is precisely the window where the
control law is active and depends on the attitude estimate being right.

Note that the bias *subtraction* (`wx -= bx`) is ungated and continues throughout. Only the
bias *estimate update* is gated by `trust`. So the pad hold's converged bias value stays
frozen and keeps being applied during boost — the drift measured below is the residual
after that subtraction, not raw gyro bias.

### Method

Temporary `DRIFT_TEST` build forces `trust = 0.0f` once the vehicle leaves `FLIGHT_DISARMED`,
reproducing the boost-phase condition without needing acceleration. Bias converges normally
during the 20 s arming hold, then correction cuts out at the `*** ARMED ***` transition.
`tilt` recorded at that transition, at +3.5 s (burn duration), and at +10 s (shape check).

### Results

| Run | At cutoff | +3.5 s | Drift | +10 s | Drift |
|---|---|---|---|---|---|
| 1 | 2.13° | 2.65° | 0.52° | 4.44° | 2.31° |
| 2 | 1.97° | 2.40° | 0.43° | 4.23° | 2.26° |
| 3 | 1.99° | 2.41° | 0.42° | 4.22° | 2.23° |

**Rate: ~0.13 °/s over the burn window, ~0.23 °/s over 10 s.**

Drift is approximately linear with a mild upward curl, consistent with uncorrected constant
gyro bias plus slow bias wander. No sign of integration blowup or quaternion normalization
trouble.

### Interpretation

**Accumulated phantom tilt at burnout: ~0.45°.** Against the 4° linear range before
saturation, that consumes roughly 11% of available headroom. Acceptable.

**The repeatability is the notable result.** Runs 2 and 3 agree to 0.01° at both
checkpoints, across separate power cycles, always in the same direction. A random-walk bias
would not repeat like that. This is a *systematic* residual — most likely bias the Mahony
integral term cannot fully null within the 20 s hold, given `Ki = 0.01` and the
`BIAS_MAX = 0.05` rad/s cap.

A predictable error is a better error. 0.45° is small enough to design around rather than
engineer against.

### Not a runaway

Drift is unbounded *in the test*, because the test never restores correction. In flight it
is bounded by burn duration: at burnout, acceleration drops, `g_err` falls back inside
`TRUST_ZERO`, trust returns, and the filter pulls the estimate back toward gravity. Drift
resets every burn and does not accumulate across the flight.

### Possible improvements (flight two, not before)

All of these attack bias convergence *during the pad hold* — boost-phase subtraction
already works correctly and is not the problem.

- Raise `Ki` above 0.01 so bias converges further within the hold
- Extend `ARM_HOLD_PASSES` beyond 20 s
- Explicitly average gyro output during the still period and subtract that, rather than
  relying solely on the integral term
- Freeze the bias estimate at launch detection

None is worth changing before this flight: it would mean modifying a validated estimator
days out, and the change would itself need revalidation.

**One caveat.** Gyro bias drifts with temperature. Over a 3.45 s burn that is negligible,
but a board that sat in the sun and then armed would learn a bias for a different thermal
state than it flies in. Not engineered around. Worth remembering if in-flight drift ever
fails to match these bench numbers.

---

## Open items

- [ ] Flight validation of all gains — nothing here is flight-confirmed
- [x] Boost-phase estimator drift characterization — measured 2026-09-03, ~0.45° over the
      burn. See the section above.
- [ ] Bias convergence improvement during pad hold (flight two)
- [ ] Consider gain scheduling against the thrust curve if fixed gains prove inadequate
- [ ] Re-measure `I` if mass, motor, or configuration changes. Inertia is not portable
      between builds.

---

## Notes for re-measurement

- Confirm motor retention before swinging. A sliding motor invalidated an earlier slew
  measurement and would corrupt the period here too — producing a plausible wrong answer
  rather than an obviously broken one.
- Measure `b` attachment-to-attachment at the rocket, not at the ceiling. The `(b/2)²` term
  makes this the most error-sensitive dimension in the calculation.
- Always run the swing mode as a calibration check before trusting a pitch mode result.
- Strings must be vertical and parallel at rest, attachment points symmetric about the CG.
  Asymmetry lets the rocket translate as well as rotate, contaminating the mode.
- Small amplitude, under ~10°. Beyond that the period lengthens and the small-angle
  formulas degrade.
- Bifilar *torsional* mode (twisting about the long axis) measures **roll** inertia, which
  is a different and much smaller number. TVC controls pitch and yaw. Do not substitute one
  for the other.
