# TVC Flight Software

Thrust vector control flight computer for a finless model rocket. Written from
scratch in C for an STM32F411CEU6, with a Mahony attitude filter, a 3-state
Kalman filter for the vertical channel, PD attitude control driving a two-axis
gimbal, servo-actuated recovery deployment, and 200 Hz flight logging to SPI
flash.

**Status: flown once.** The vehicle launched, was actively stabilised through
boost, deployed under parachute at apogee, and was recovered. See
[Flight 1](flight_1/) for the log and analysis, including what went wrong.

---

## Hardware

| Part | Role |
|---|---|
| STM32F411CEU6 "Black Pill" | Flight computer, 100 MHz |
| ISM330DHCX | 6-axis IMU (accel + gyro), SPI |
| BMP280 | Barometer, SPI |
| W25Q128 | 16 MB SPI flash, flight log |
| 2× SG90 | TVC gimbal servos |
| 1× SG90 | Recovery latch release |
| Estes F15-0 | Motor (plugged, no ejection charge) |

Pin assignments, servo calibration, and clock configuration are in
[CALIBRATION.md](TVC_Flight_Software/CALIBRATION.md).

---

## Features

**Attitude estimation.** A Mahony complementary filter fuses gyro and
accelerometer into a quaternion. The accelerometer's weight fades to zero when
total acceleration departs from 1 g, which is correct under thrust — the
accelerometer's "down" points backward along the body axis during boost, so
correcting toward it would corrupt the estimate. Attitude therefore runs on
open-loop gyro integration for the whole burn; measured drift is ~0.45° over
3.45 s.

**Vertical channel.** A 3-state Kalman filter (altitude, velocity, accelerometer
bias) predicts at 200 Hz from the accelerometer and corrects at 25 Hz from the
barometer. Drives burnout and apogee detection.

**Control.** PD on attitude error, gains derived from a measured plant model —
moment of inertia from a bifilar pendulum test, lever arm and CG measured
directly. Commands are clamped to ±6° of gimbal and slew-limited to 25 µs of
pulse change per tick. Active during boost only. See
[TUNING.md](TVC_Flight_Software/TUNING.md).

**State machine.** DISARMED → PAD → BOOST → COAST → DESCENT, each transition
debounced 100 ms. Arming requires 20 s of stillness and near-vertical attitude,
then both servos wiggle in sequence so the vehicle can signal "armed" with no
serial connection attached.

**Recovery.** A servo knocks a spring-loaded latch. Primary trigger is apogee
detection; a backup timer fires 7.5 s after launch detection regardless of
state, because every path to apogee runs through launch and burnout detection
and any one of them failing leaves the vehicle ballistic.

**Logging.** 78-byte records at 200 Hz — quaternion, body rates, raw
accelerometer, attitude error, commands, servo pulses, filter state, flight
state, and a flag byte. Three records per 256-byte flash page. Records are never
allowed to straddle a page boundary, so a truncated write can't leave a
half-record that's hard to detect.

---

## Repository Layout

    TVC_Flight_Software/     firmware -- CubeMX project, CMake build
      Core/Src/main.c        everything lives here
      tools/                 Python log decoder
      CALIBRATION.md         pins, servo geometry, slew rate, clock tree
      TUNING.md              plant model, moment of inertia, gain derivation
      PREFLIGHT.md           pre-flight checklist and arming procedure
    flight_1/                log, replay, and post-flight analysis
    README.md

---

## Building

Requires STM32CubeMX, the ARM GCC toolchain, and CMake.

    cd TVC_Flight_Software
    cmake --preset Debug
    cmake --build build/Debug

Flash the resulting `.elf` with STM32CubeProgrammer or the CubeIDE extension.

**Before flashing for flight, check every test flag is `0`** — `BENCH_TEST`,
`SLEW_TEST`, `DRIFT_TEST`, `SIM_MODE`, `EJECTION_BENCH_TEST`. Several of them
fail silently. `SLEW_TEST` in particular enters an infinite servo-sweep loop
before the flight loop is ever reached, so the board looks alive and does
nothing. [PREFLIGHT.md](TVC_Flight_Software/PREFLIGHT.md) covers this properly.

---

## Reading a flight log

Capture the flash dump over serial, then:

    cd TVC_Flight_Software/tools
    python decode_flight_log.py capture.log --replay

Full instructions in
[tools/README.md](TVC_Flight_Software/tools/README.md).

---

## Status

The vehicle flew, stabilized, deployed, and was recovered, but the control
loop is not fully validated.

Flight 1 left a crooked rail roughly 7-10° off vertical, which saturated the
controller before the rocket cleared the rod. The gains clamp at 4° of attitude
error, so the loop spent ~80% of the burn pinned at full deflection, running
bang-bang rather than PD. It held the vehicle — tilt stayed bounded and it never
departed — but nothing about `Kp_att` or `Kd_att` was actually exercised.

So the gains are **computed from a measured plant, not validated in flight.**
Flight 2 is a straight-rail repeat with unchanged firmware, specifically so the
loop gets to operate in its linear range and the log becomes tunable.

Also unresolved: the vehicle rolled at up to 500 °/s, beginning while still on
the rod. A two-axis gimbal cannot correct roll, and roll cross-couples the pitch
and yaw channels through actuator lag — at 500 °/s the body rotates 25° during
the servo's ~50 ms transit, so corrections land rotated from where they were
computed. Traced to loose, closely-spaced rail buttons on a finless vehicle with
no aerodynamic roll damping.

---

## License

MIT
