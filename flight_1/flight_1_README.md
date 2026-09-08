# Flight 1

**Date:** 9/7/2026
**Location:** Holtville Airport, CA
**Motor:** Estes F15-0 (plugged — no ejection charge, recovery is servo-actuated)
**Vehicle mass:** 701 g, flight configuration
**Result:** Launched, actively stabilised, deployed at apogee, recovered intact

---

## Files

| File | What it is |
|---|---|
| `capture.log` | Raw PuTTY capture of the flash dump. The irreplaceable artifact — everything else regenerates from this. |
| `flight1.gif` | 3D attitude replay from the decoder |

Decode with:

    cd ../TVC_Flight_Software/tools
    python decode_flight_log.py ../../flight_1/capture.log --replay

---

## Software as flown

Firmware was unchanged from the initial commit of this repository. The only
subsequent change was renaming the CubeMX project, which did not touch
`main.c`.

| Constant | Value |
|---|---|
| `Kp_att` | 86.0 |
| `Kd_att` | 10.0 |
| `MAX_DEFLECT` | 6.0° |
| `SLEW_MAX_US` | 25 |
| `EJECT_BACKUP_PASSES` | 1500 (7.5 s after launch detection) |
| `SERVO_MY_SIGN` | −1.0 |
| `SERVO_MX_SIGN` | +1.0 |

Design targets behind those gains: ω_n = 12 rad/s, ζ = 0.7, from a measured
plant (I = 0.0668 kg·m², lever arm 0.429 m). See `../TUNING.md`.

---

## What the log shows

**State timeline** (times relative to log start; the vehicle sat armed on the
pad for 51 s before launch):

| Time | State |
|---|---|
| 0.00 s | PAD |
| 51.05 s | BOOST |
| 53.65 s | COAST |
| 55.09 s | DESCENT — ejection fired on apogee detection |

| Metric | Value |
|---|---|
| Max altitude | 55.9 m |
| Max vertical velocity | 31.1 m/s |
| Detected burn duration | 2.60 s |
| Max tilt | 126.4° (during descent, after ejection) |
| Loop rate | 200 Hz, dt 5.00 ms mean, no overruns |

Flag counts, and why the whole-log percentage is misleading — the log is mostly
pad time:

| Flag | Of log | Of burn |
|---|---|---|
| SAT_Y | 3.2% | ~80% |
| SAT_Z | 3.3% | ~83% |
| NO_TRUST | 11.4% | 100% (expected — see `../TUNING.md`) |
| OVERRUN | 0 | 0 |
| PULSE_Y / PULSE_Z | 0 | 0 |

---

## What went wrong

### The rail was crooked

The dominant problem, and the cause of nearly everything else.

At launch detection the vehicle was already **~10° off vertical** — `err_y` at
−7° and `err_z` at +6° in the PAD records immediately preceding BOOST. With
`Kp_att` = 86, the controller saturates at 4° of attitude error, so it was
pinned at ±6° of gimbal within one or two ticks of the motor lighting. It never
had a linear phase.

**Consequence: the gains were never exercised.** For ~80% of the burn the
output was the clamp, not `Kp·err + Kd·ω`. The loop ran bang-bang. It held the
vehicle — tilt cycled roughly 2° → 26° → 8° → 18° and stayed bounded, and TVC
visibly recovered the vehicle from the off-rail pitchover — but nothing in
`TUNING.md` was validated by this flight.

### Roll

`wx` reached ~500 °/s during boost and railed the ±1000 °/s gyro full scale
during descent. Video confirms the vehicle was **rotating while still on the
rod**, then aggressively as it released.

Traced to loose rail buttons spaced too closely — one ~2" from the aft end, one
~2" below the CG. On a finless vehicle there is no aerodynamic roll damping, so
induced roll persists and grows.

Roll matters beyond being untidy: a two-axis gimbal cannot correct it, and it
cross-couples the pitch and yaw channels. The servo takes ~50 ms to traverse,
during which a body rolling at 500 °/s rotates **25°** — so each correction
lands rotated from the frame it was computed in, reducing effective authority
by cos(25°) and bleeding Y commands into Z.

### Altitude loss

55.9 m against an OpenRocket prediction substantially higher. The off-vertical
departure spent thrust on horizontal velocity rather than altitude.

### Melted motor liner tape

The tape securing the motor liner to the TVC body tube melted during the burn,
making the lower airframe unusable for a second flight. Adhesive near the motor
casing is not a viable retention method.

---

## What worked

- **Recovery deployed on apogee detection**, not the backup timer. `EJECT_BAK`
  never set. The servo latch released cleanly and the vehicle came down under
  parachute.
- **Plugged motor held** — no case failure.
- **Logging: flawless.** 13,017 records at exactly 5.00 ms, zero overruns, zero
  dropped samples, full decode on the first attempt.
- **TVC demonstrably worked.** The vehicle pitched over off the rail and the
  controller pulled it back toward vertical. Saturated and inelegant, but it
  did the job it exists to do.
- **State machine** progressed correctly through all four transitions with no
  spurious triggers.

---

## Changes for flight 2

**Mechanical only. Firmware unchanged.**

1. **Straight, verified-level launch rail.** This is the whole flight. A 10°
   initial condition is what saturated the controller.
2. **Rail buttons: wider spacing, tighter fit to the rod.** Addresses the roll
   at its source.
3. **Reprinted TVC assembly and lower airframe**, with motor retention that
   does not depend on adhesive near the casing.

**Gains stay at 86 / 10.** Changing them now would be guessing against no
evidence — the loop never operated where they apply. Fixing the rail and flying
the same firmware produces a flight where the controller runs in its linear
range, and *that* log is the one that makes the gains tunable.

Verify mass and CG are unchanged after the reprint (701 g, CG 488 mm from aft).
If either moved appreciably, the plant model and therefore the gains need
revisiting.

---

## What to look for in flight 2

- **Saturation dropping to occasional** rather than continuous means the loop
  is finally linear and `TUNING.md` becomes testable.
- **Oscillation without saturation** is the clean underdamping signal — measure
  the period against the designed 2π/ω_n. Longer period means the real plant is
  slower than modelled; correct period with excess overshoot means `Kd_att` is
  low.
- **Still saturating on a straight rail** would mean the disturbance exceeds
  6° of control authority, which is a vehicle problem rather than a gain
  problem.
- **`wx` early in the burn** — confirm the rail button fix actually killed the
  roll.
