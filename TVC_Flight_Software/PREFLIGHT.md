# Pre-Flight Checklist

**Vehicle:** TVC flight computer, STM32F411CEU6
**Motor:** Estes F15-0
**Companion docs:** `CALIBRATION.md` (pins, servo geometry, slew), `TUNING.md` (plant model, gains)

Work top to bottom. Do not skip ahead — several steps exist because a later step would
otherwise hide a failure.

---

## A. Software configuration (before flashing)

Verify every test flag is `0`. Any one of these left at `1` produces a vehicle that will
not fly correctly, and most fail *silently*.

- [ ] `BENCH_TEST 0` — at `1`, the control law runs in every state including PAD
- [ ] `SLEW_TEST 0` — at `1`, code enters an infinite servo-sweep loop and **never reaches
      the flight loop**
- [ ] `DRIFT_TEST 0` — at `1`, accelerometer correction is disabled after arming
- [ ] `SIM_MODE 0` — at `1`, real accelerometer is ignored for vertical channel
- [ ] `EJECTION_BENCH_TEST 0` — at `1`, fires the ejection servo and halts

Verify flight constants match the docs:

- [ ] `Kp_att = 86.0f`, `Kd_att = 10.0f`
- [ ] `SLEW_MAX_US = 25`
- [ ] `EJECT_BACKUP_PASSES = 1500` (7.5 s after launch detection)
- [ ] `SERVO_MY_SIGN`, `SERVO_MX_SIGN` match the last bench sign verification
- [ ] `MAX_DEFLECT = 6.0f`

- [ ] Flash the board. Confirm the build actually uploaded — do not assume.

---

## B. Bench verification (at home, before leaving)

- [ ] Power on with USB-TTL connected. Confirm all three bring-up lines:
  - `ISM330DHCX IMU WHO_AM_I: 0x6B`
  - `BMP280 Chip ID: 0x58`
  - `W25Q128 JEDEC: EF 40 18`
- [ ] `LogRecord size: 78 bytes`
- [ ] **Erase the flash.** Press `e`, then `y` at the ground menu. See note in section F on
      why this matters.
- [ ] Power cycle. Confirm `Log write pointer: 0x000000`.
- [ ] Let it arm (20 s still and upright). Confirm `*** ARMED ***` and the servo wiggle:
      Y channel four times, pause, Z channel four times, center.
- [ ] Confirm the wiggle looks *smooth*, not snappy — that is the slew limiter working.
- [ ] Watch `[timing]` for several seconds. `worst` should stay well under 5000 µs;
      `overruns` should be `0`.
- [ ] Disarm by powering down.

**Battery**

- [ ] LiPo charged
- [ ] Connector secure, no strain on the wire
- [ ] Servo power common ground confirmed

---

## C. Vehicle assembly

- [ ] **Motor retained.** Not friction alone — the retention must survive thrust and
      ejection. A sliding motor shifts the CG and lever arm mid-burn and invalidates the
      entire plant model in `TUNING.md`.
- [ ] Gimbal moves freely through full travel by hand, power off. Nothing fouls.
- [ ] No slop in the linkage. Backlash adds effective actuator lag on top of servo transit.
- [ ] Recovery system packed, latch engaged, shock cord clear of the gimbal
- [ ] Barometer static port clear and unobstructed
- [ ] All wiring secured, connectors seated, nothing that can shift under 2 g
- [ ] Tape any servo connectors to their extenders at connection point to avoid accidental disconnect in flight
- [ ] Board mounted solidly — IMU orientation must match what the sign check assumed

---

## D. At the pad

- [ ] Range safety officer briefed and clear
- [ ] Rocket on the rail, vertical
- [ ] Igniter installed, leads shorted or disconnected until ready
- [ ] Recovery area clear

**Arming sequence**

- [ ] Power on the flight computer
- [ ] Skip the ground menu (wait 3 s, or press any key that is not `d` or `e`)
- [ ] **Leave the vehicle still and upright for the full 20 s arming hold.** This is when
      the gyro bias estimate converges. A vehicle jostled during this window arms with a
      worse bias and drifts more during the burn.
- [ ] **Confirm the wiggle.** Y channel, then Z channel. This is the vehicle's only
      signal that it armed successfully with no serial connection.
  - No wiggle → it did not arm. Do not launch. Power down and restart the hold.
  - Wiggle on one channel only → dead servo channel. Do not launch.
- [ ] Step back. Vehicle is now live and will detect launch on acceleration.

---

## E. Abort

If anything goes wrong before ignition:

- [ ] **Disconnect power.** This is the only abort. There is no software disarm.

Powering down safes everything: PWM stops, servos go inert, the ejection servo's compare
register is reloaded to `EJECT_ARMED_US` on the next boot before the channel is enabled.

**If the igniter is already connected**, follow range procedure for approaching a live pad
before touching the vehicle.

**Re-arming after an abort** requires a full power cycle and another 20 s hold. Budget for
this — do not rush the second hold to make up time.

---

## F. Post-flight

- [ ] Recover the vehicle. Note anything visibly wrong before touching it.
- [ ] **Do not power cycle in the field** unless you have to. The log is in non-volatile
      flash and will survive, but every boot risks an accidental keypress at the ground
      menu.
- [ ] At home: connect USB-TTL, press `d` at the ground menu, capture the dump to a file
- [ ] Run the decoder. Confirm the state timeline shows PAD → BOOST → COAST → DESCENT.
- [ ] Archive the raw capture before doing anything else with the flash.
- [ ] **Then** erase, before the next flight.

**Why erase matters.** Logging starts at PAD, not at BOOST — so the flash fills at
17 KB/s from the moment the vehicle arms. The 16 MB chip holds roughly **16 minutes of
armed pad time**. A long range hold on a partially-full chip can exhaust the space before
the motor lights, and the flight is lost. The `log_addr + 256 <= FLASH_SIZE` guard stops
the write cleanly rather than corrupting anything, but stopping cleanly still means no
flight data.

Corollary: **do not arm early.** Arm when you are close to launching.

---

## G. What to look for in the first flight log

Nothing in `TUNING.md` has been flight-validated. This flight is the validation.

| Check | What it means |
|---|---|
| `cmd_y`/`cmd_z` vs `pulse_y`/`pulse_z` | Constant divergence through boost = slew limiter binding = gains too aggressive for the actuator |
| `LOG_FLAG_SAT_Y` / `SAT_Z` frequency | Occasional is fine. Continuous means `Kp_att` too high or disturbances larger than expected |
| `LOG_FLAG_NO_TRUST` | Should be set for essentially the whole burn. This is correct — see the boost-phase drift section in `TUNING.md` |
| `LOG_FLAG_OVERRUN` | Should be zero. Any occurrence means the loop missed its 5 ms deadline |
| `LOG_FLAG_PULSE_Y` / `PULSE_Z` | Should never fire. If set, a software error drove a servo to its mechanical bound |
| Attitude response shape | Overshoot and ring = `Kd_att` too low or `Kp_att` too high. Sluggish with no overshoot = room to raise `ω_n` |
| Tilt at burnout | Includes ~0.45° of expected gyro drift (measured). Larger suggests real attitude error |
| Which ejection trigger fired | `EJECT_BAK` set means apogee detection failed and the backup timer saved it. Investigate before flying again |

---

## Known limitations going into flight one

Stated plainly so nothing here is a surprise:

- **Gains are computed, not validated.** ω_n = 12 rad/s, ζ = 0.7, derived from a measured
  plant. Never flown.
- **Thrust varies ±40% across the burn**, so the plant the gains were tuned against exists
  only at one instant. The loop is faster early, slower late.
- **Servo slew under thrust load was never measured** — only under inertial load with an
  inert motor. Bearing friction and nozzle aero load are bounded by argument, not data.
- **Attitude runs open-loop on gyro integration for the entire burn**, accumulating ~0.45°.
  Measured on the bench, never in flight.
- **There is no software abort.** Power disconnection is the only way to safe an armed
  vehicle.
