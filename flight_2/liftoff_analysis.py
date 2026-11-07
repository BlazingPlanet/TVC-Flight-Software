"""Flight 2 liftoff analysis.

Reproduces the figures and numbers in flight_2/README.md from the decoder's
CSV output. Nothing here uses the gyro to estimate attitude except where it
is explicitly compared against the gyro-free physics replay.

Requirements:
    python -m pip install numpy pandas matplotlib

Usage, from the repo root (PowerShell):
    cd flight_2
    python ..\\TVC_Flight_Software\\tools\\decode_flight_log.py capture.log --flight 1 --no-trim --no-plots --csv flight2.csv
    python liftoff_analysis.py flight2.csv

--flight 1 is required: this dump holds two sessions, and Flight 2 is the
decoder's "Flight 1" (27309 records). Its "Flight 2" is a short pad-only
session recorded afterwards.

Outputs liftoff_detail.png and liftoff_replay.png in the current directory
and prints gyro statistics, the actuator sign check, and the replay peak.
"""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---- Vehicle and servo constants (as flown) --------------------------------
MASS = 0.705      # kg, loaded
LEVER = 0.429     # m, gimbal pivot to CG
INERTIA = 0.0668  # kg m^2, pitch/yaw, from bifilar pendulum
G = 9.80665
DT = 0.005

MY_TRIM, MY_USPD, MY_SIGN = 1600, 44.4, -1.0
MX_TRIM, MX_USPD, MX_SIGN = 1875, 48.5, +1.0

BOOST = 2


def gimbal_deg(d):
    """Actual gimbal angle in command convention, recovered from the pulse.

    The pulse is what the servo was told after slew limiting, so this is the
    nozzle angle minus the servo's own transit lag."""
    dy = (d.pulse_y - MY_TRIM) / (MY_SIGN * MY_USPD)
    dz = (d.pulse_z - MX_TRIM) / (MX_SIGN * MX_USPD)
    return dy.values, dz.values


def physics_replay(d, t0, t1, lag_ms):
    """Integrate rigid-body pitch/yaw from logged gimbal angle and thrust.

    No gyro input. Thrust is taken from the axial accelerometer (specific
    force x mass), which is valid at low speed where drag is small. Ignores
    roll coupling and aerodynamics, so it is a lower bound on the real
    motion rather than a reconstruction of it."""
    t = d.t.values
    dy, dz = gimbal_deg(d)
    thrust = np.clip(d.ax_g.values, 0, None) * MASS * G
    lag = lag_ms // 5
    i0, i1 = np.searchsorted(t, t0), np.searchsorted(t, t1)
    wy = wz = ay = az = 0.0
    tilt, rate = [], []
    for i in range(i0, i1):
        j = max(i - lag, 0)
        k = thrust[i] * LEVER / INERTIA
        wy += k * np.radians(dy[j]) * DT
        wz += k * np.radians(dz[j]) * DT
        ay += wy * DT
        az += wz * DT
        tilt.append(np.degrees(np.hypot(ay, az)))
        rate.append(np.degrees(np.hypot(wy, wz)))
    return t[i0:i1], np.array(tilt), np.array(rate)


def gyro_stats(d):
    """Mean rate, and high-frequency content (what is left after removing a
    50 ms moving average). Rigid-body motion lives in the moving average;
    vibration lives in the residual."""
    t = d.t.values
    w = np.degrees(d[["wx", "wy", "wz"]].values)
    smooth = pd.DataFrame(w).rolling(10, center=True).mean().values
    hf = w - smooth
    phases = [
        ("Pad, still", 100.0, 121.80),
        ("Ignition, still on pad", 121.86, 121.91),
        ("Boost, first 0.3 s", 122.00, 122.30),
        ("Coast, no thrust", 124.70, 126.00),
    ]
    print("\nGyro by phase (deg/s, x / y / z)")
    for name, a, b in phases:
        s = (t > a) & (t < b)
        print(f"  {name:24s} mean {np.round(w[s].mean(0), 1)}  "
              f"HF std {np.round(np.nanstd(hf[s], 0), 1)}")
    dw = np.abs(np.diff(w[(t > 122.0) & (t < 122.6)], axis=0)).max()
    print(f"  Largest tick-to-tick change, boost: {dw:.0f} deg/s "
          f"({dw / DT:.0f} deg/s^2)")


def sign_check(d):
    """Lateral accelerometer vs gimbal angle. A correctly signed actuator
    produces az with the same sign as Y deflection and ay with the opposite
    sign to Z deflection (thrust acts aft of the CG)."""
    t = d.t.values
    dy, dz = gimbal_deg(d)
    lag = 8  # 40 ms servo transit
    s = (d.state.values == BOOST) & (t < 122.6)
    s[:lag] = False
    idx = np.nonzero(s)[0]
    cy = np.corrcoef(d.az_g.values[idx], dy[idx - lag])[0, 1]
    cz = np.corrcoef(d.ay_g.values[idx], dz[idx - lag])[0, 1]
    print(f"\nActuator sign check, first 0.6 s of boost:")
    print(f"  corr(az, Y gimbal) = {cy:+.2f}   (expect positive)")
    print(f"  corr(ay, Z gimbal) = {cz:+.2f}   (expect negative)")


def plot_detail(d, path):
    t = d.t.values
    s = (t > 121.80) & (t < 122.60)
    dy, dz = gimbal_deg(d)
    boost_t = t[np.argmax(d.state.values == BOOST)]

    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(t[s], d.ax_g.values[s], color="k")
    ax[0].set_ylabel("Axial accel (g)")
    ax[0].set_title("Thrust (axial specific force)")

    for c, col in (("wx", "C0"), ("wy", "C1"), ("wz", "C2")):
        ax[1].plot(t[s], np.degrees(d[c].values[s]), color=col, label=c, lw=1)
    ax[1].set_ylabel("Rate (deg/s)")
    ax[1].set_title("Logged gyro rates")
    ax[1].legend(loc="upper left", ncol=3)

    ax[2].plot(t[s], d.cmd_y.values[s], color="C0", lw=0.8, alpha=0.6,
               label="cmd_y (commanded)")
    ax[2].plot(t[s], dy[s], color="C0", lw=2, label="Y gimbal (from pulse)")
    ax[2].set_ylabel("Y (deg)")
    ax[2].set_title("Y axis: command vs slew-limited gimbal")
    ax[2].legend(loc="upper right")

    ax[3].plot(t[s], d.cmd_z.values[s], color="C2", lw=0.8, alpha=0.6,
               label="cmd_z (commanded)")
    ax[3].plot(t[s], dz[s], color="C2", lw=2, label="Z gimbal (from pulse)")
    ax[3].set_ylabel("Z (deg)")
    ax[3].set_title("Z axis: command vs slew-limited gimbal")
    ax[3].legend(loc="upper right")
    ax[3].set_xlabel("Log time (s)")

    for a in ax:
        a.axvline(boost_t, color="r", ls="--", lw=1)
        a.grid(alpha=0.3)
    ax[0].text(boost_t + 0.005, ax[0].get_ylim()[1] * 0.9, "BOOST",
               color="r")
    fig.suptitle("Flight 2 liftoff, 121.8 to 122.6 s")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"Wrote {path}")


def plot_replay(d, path):
    t = d.t.values
    fig, ax = plt.subplots(figsize=(10, 5))
    for lag, ls in ((0, ":"), (40, "-"), (80, "--")):
        tt, tilt, _ = physics_replay(d, 121.90, 122.70, lag)
        ax.plot(tt, tilt, color="C3", ls=ls,
                label=f"Physics replay, {lag} ms servo lag")
    s = (t > 121.90) & (t < 122.70)
    ax.plot(t[s], d.tilt_deg.values[s], color="k", lw=2,
            label="Logged tilt (gyro-integrated)")
    ax.set_xlabel("Log time (s)")
    ax.set_ylabel("Tilt (deg)")
    ax.set_title("Tilt the nozzle commands should produce vs tilt the gyro "
                 "reported")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"Wrote {path}")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    d = pd.read_csv(sys.argv[1])
    gyro_stats(d)
    sign_check(d)
    tt, tilt, rate = physics_replay(d, 121.90, 122.70, 40)
    k = np.argmax(tilt)
    print(f"\nPhysics replay (40 ms lag): peak {tilt[k]:.0f} deg at "
          f"t = {tt[k]:.2f} s, peak rate {rate.max():.0f} deg/s")
    plot_detail(d, "liftoff_detail.png")
    plot_replay(d, "liftoff_replay.png")


if __name__ == "__main__":
    main()
