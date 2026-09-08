#!/usr/bin/env python3
"""
TVC Flight Log Decoder
======================

Decodes a PuTTY capture of the flight computer's flash dump into CSV and plots.

USAGE
-----
    python decode_flight_log.py capture.log
    python decode_flight_log.py capture.log --csv flight.csv
    python decode_flight_log.py capture.log --replay      # 3D orientation replay
    python decode_flight_log.py capture.log --flight 2    # pick one flight
    python decode_flight_log.py capture.log --no-trim     # plot the whole log
    python decode_flight_log.py capture.log --save-replay flight.gif

REQUIREMENTS
------------
    pip install numpy matplotlib
    (--save-replay to .mp4 also needs ffmpeg on PATH; .gif does not)

WHAT IT EXPECTS
---------------
A text file containing the firmware's dump output. Boot messages before and
after the markers are ignored, so you can just log the whole PuTTY session:

    ---FLASH DUMP BEGIN---
    CB9E0000E9D37F3F72F969BC...          <- 512 hex chars = one 256-byte page
    D09E0000EFD37F3FDAF269BC...
    ---FLASH DUMP END--- 579328 bytes

PAGE LAYOUT
-----------
Each 256-byte flash page holds three 78-byte records (234 bytes) followed by
22 bytes of 0xFF padding. The firmware does it this way because records don't
tile evenly into pages, and letting a record straddle a page boundary would
mean a truncated write could leave a half-record that's hard to detect.
Wasting 22 bytes per page costs 8.6% of a 16 MB chip -- irrelevant here.
"""

import argparse
import struct
import sys

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Record format
# ---------------------------------------------------------------------------
#
# This MUST match the packed struct in main.c exactly. If you add a field to
# LogRecord, add it here too, in the same position.
#
#   typedef struct __attribute__((packed)) {
#     uint32_t t_ms;                  // 4
#     float qw, qx, qy, qz;           // 16
#     float wx, wy, wz;               // 12  post-bias-correction, rad/s
#     float ax_g, ay_g, az_g;         // 12  raw accel, g
#     float err_y, err_z;             // 8
#     float cmd_y, cmd_z;             // 8   gimbal degrees
#     uint16_t pulse_y, pulse_z;      // 4   microseconds
#     float kf_h, kf_v, kf_b;         // 12
#     uint8_t state;                  // 1
#     uint8_t flags;                  // 1
#   } LogRecord;                      // 78 bytes
#
# '<' means little-endian AND no alignment padding, which matches the
# __attribute__((packed)) on the C side. Without '<', Python would insert
# padding the same way an unpacked C struct would, and every field after the
# uint16 pair would decode as garbage.

RECORD_FMT = "<I" + "f" * 4 + "f" * 3 + "f" * 3 + "f" * 2 + "f" * 2 + "HH" + "f" * 3 + "BB"
RECORD_SIZE = struct.calcsize(RECORD_FMT)

RECORDS_PER_PAGE = 3
PAGE_SIZE = 256

# Sanity check at import time. If this fires, the format string and the C
# struct have drifted apart and nothing downstream can be trusted.
assert RECORD_SIZE == 78, f"Record format is {RECORD_SIZE} bytes, expected 78"

FIELD_NAMES = [
    "t_ms",
    "qw", "qx", "qy", "qz",
    "wx", "wy", "wz",
    "ax_g", "ay_g", "az_g",
    "err_y", "err_z",
    "cmd_y", "cmd_z",
    "pulse_y", "pulse_z",
    "kf_h", "kf_v", "kf_b",
    "state", "flags",
]

STATE_NAMES = ["DISARM", "PAD", "BOOST", "COAST", "DESCENT"]

# State byte values, referenced by trim_flight().
STATE_BOOST = 2
STATE_DESCENT = 4

# Flag bits, mirroring the #defines in main.c
FLAG_BITS = [
    (0x01, "SAT_Y",    "cmd_y clamped at MAX_DEFLECT"),
    (0x02, "SAT_Z",    "cmd_z clamped at MAX_DEFLECT"),
    (0x04, "PULSE_Y",  "pulse_y hit us bound -- SHOULD NEVER FIRE"),
    (0x08, "PULSE_Z",  "pulse_z hit us bound -- SHOULD NEVER FIRE"),
    (0x10, "NO_TRUST", "accel trust was zero (expected during boost)"),
    (0x20, "OVERRUN",  "loop missed its 5 ms deadline"),
    (0x40, "EJECT",     "ejection sequence active"),
    (0x80, "EJECT_BAK", "fired by backup timer, not apogee"),
]

GRAVITY = 9.80665  # m/s^2, must match the firmware


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def extract_hex(path):
    """Pull the hex payload out of a PuTTY capture.

    Returns a single bytes object containing every dumped page, concatenated.

    We scan for the markers rather than assuming the file starts with hex,
    because the capture will contain the boot banner, the JEDEC ID line, the
    menu prompt, and so on. Anything outside the markers is ignored.
    """
    with open(path, "r", errors="replace") as f:
        lines = f.read().splitlines()

    start = end = None
    for i, line in enumerate(lines):
        if "---FLASH DUMP BEGIN---" in line:
            start = i + 1
        elif "---FLASH DUMP END---" in line:
            end = i
            break

    if start is None:
        sys.exit("No '---FLASH DUMP BEGIN---' marker found. "
                 "Was PuTTY logging enabled when you pressed 'd'?")
    if end is None:
        # The capture was cut short -- take what we have rather than bailing.
        print("WARNING: no END marker; the dump may be truncated.")
        end = len(lines)

    payload = "".join(line.strip() for line in lines[start:end])

    # Guard against stray non-hex characters (terminal noise, partial lines).
    payload = "".join(c for c in payload if c in "0123456789abcdefABCDEF")

    if len(payload) % 2:
        print("WARNING: odd number of hex characters; dropping the last one.")
        payload = payload[:-1]

    return bytes.fromhex(payload)


def parse_records(raw):
    """Slice raw flash bytes into records, honouring the page layout.

    Walks page by page, takes the first 234 bytes of each, and unpacks three
    records from them. The trailing 22 bytes of padding are skipped.

    A record that is entirely 0xFF is an erased slot -- that happens on the
    final page if the flight ended mid-page, and it means we've reached the
    end of the data.
    """
    records = []
    n_pages = len(raw) // PAGE_SIZE

    if len(raw) % PAGE_SIZE:
        print(f"WARNING: {len(raw) % PAGE_SIZE} trailing bytes are not a whole "
              f"page; ignoring them.")

    for p in range(n_pages):
        page = raw[p * PAGE_SIZE:(p + 1) * PAGE_SIZE]

        for r in range(RECORDS_PER_PAGE):
            chunk = page[r * RECORD_SIZE:(r + 1) * RECORD_SIZE]

            # Erased slot -> end of data.
            if all(b == 0xFF for b in chunk):
                return records

            records.append(struct.unpack(RECORD_FMT, chunk))

    return records


def split_flights(records):
    """Split concatenated flights on the timestamp resetting.

    t_ms comes from HAL_GetTick(), which restarts at zero on every boot. Since
    the flash is append-only across power cycles, a timestamp that jumps
    *backwards* marks the boundary between one flight's data and the next.

    This costs nothing -- no header, no marker in the format -- and it falls
    out of a field we wanted anyway.
    """
    if not records:
        return []

    flights = [[records[0]]]
    for prev, cur in zip(records, records[1:]):
        if cur[0] < prev[0]:          # t_ms went backwards
            flights.append([])
        flights[-1].append(cur)

    return flights


# ---------------------------------------------------------------------------
# Derived quantities
# ---------------------------------------------------------------------------

def quat_rotate(q, v):
    """Rotate vector(s) v from body frame to world frame by quaternion(s) q.

    Vectorised over time: q is (N,4) as [w,x,y,z], v is (N,3), result is (N,3).

    This is the same operation as quat_rotate_vector() in the firmware, using
    the standard expansion of q (x) (0,v) (x) q* into vector algebra:

        v' = v + 2 * w * (u x v) + 2 * (u x (u x v))

    where u is the vector part of q. Mathematically identical to the two
    quaternion multiplies the C code does, just cheaper and easier to
    vectorise here.
    """
    w = q[:, 0:1]
    u = q[:, 1:4]
    uv = np.cross(u, v)
    return v + 2.0 * w * uv + 2.0 * np.cross(u, uv)


def compute_derived(data):
    """Add columns the firmware computed but didn't log.

    Linear acceleration is the interesting one. The record stores the raw
    accelerometer (body frame, in g) and the quaternion, which is everything
    needed to reconstruct lin_accel_z exactly as the firmware did:

        rotate body accel into the world frame, subtract 1 g of gravity.

    This is the concrete payoff of logging authoritative state instead of
    derived values. If you ever suspect the firmware's gravity subtraction or
    its frame conventions, you can re-derive them here and compare.
    """
    q = np.column_stack([data["qw"], data["qx"], data["qy"], data["qz"]])
    a_body = np.column_stack([data["ax_g"], data["ay_g"], data["az_g"]])

    a_world = quat_rotate(q, a_body)

    data["wax_g"] = a_world[:, 0]
    data["way_g"] = a_world[:, 1]
    data["waz_g"] = a_world[:, 2]

    # World +Z is up in this estimator, so a stationary sensor reads +1 g.
    data["lin_accel_z"] = (a_world[:, 2] - 1.0) * GRAVITY

    # Tilt from vertical: rotate world-up into the body frame and take the
    # angle between it and the nose (body +X). Same as attitude_tilt().
    up_world = np.tile([0.0, 0.0, 1.0], (len(q), 1))
    q_conj = q.copy()
    q_conj[:, 1:4] *= -1.0
    up_body = quat_rotate(q_conj, up_world)

    ux = np.clip(up_body[:, 0], -1.0, 1.0)
    data["tilt_deg"] = np.degrees(np.arccos(ux))

    # Total body rate magnitude -- useful for spotting tumbles.
    data["gyro_mag"] = np.sqrt(data["wx"]**2 + data["wy"]**2 + data["wz"]**2)

    return data


def to_arrays(flight):
    """Turn a list of unpacked tuples into a dict of numpy arrays."""
    cols = list(zip(*flight))
    data = {name: np.array(col) for name, col in zip(FIELD_NAMES, cols)}

    # Seconds since the first record, which is far more readable than raw ms.
    data["t"] = (data["t_ms"] - data["t_ms"][0]) / 1000.0

    return compute_derived(data)


def trim_flight(data, pre_boost=10.0, post_descent=10.0):
    """Slice the log down to the window worth looking at.

    Logging starts at PAD, so a log is mostly pad time -- a five minute hold
    followed by fifteen seconds of flight squeezes the entire powered portion
    into a few pixels. This keeps `pre_boost` seconds before BOOST entry
    through `post_descent` seconds after DESCENT entry.

    Returns (data, (t_first, t_last)) or (data, None) when there is nothing
    to trim. No BOOST in the log means the vehicle never launched, and in that
    case you want to see everything -- the reason it didn't launch is in the
    part that would have been cut.
    """
    state = data["state"]
    t = data["t"]

    boost = np.where(state == STATE_BOOST)[0]
    if not len(boost):
        return data, None

    t_start = t[boost[0]] - pre_boost

    # No DESCENT means apogee was never detected -- run to the end of the log
    # rather than guessing where the interesting part stops.
    descent = np.where(state == STATE_DESCENT)[0]
    t_end = t[descent[0]] + post_descent if len(descent) else t[-1]

    keep = (t >= t_start) & (t <= t_end)
    if keep.sum() < 2:
        return data, None

    trimmed = {k: v[keep] for k, v in data.items()}
    return trimmed, (t[keep][0], t[keep][-1])


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_summary(data):
    """Print flight statistics and anything worth investigating.

    Always runs on the full log, never the trimmed one: flag percentages and
    record counts should describe the flight as flown, not the plot window.
    """
    t = data["t"]
    print(f"\n  Records:  {len(t)}")
    print(f"  Duration: {t[-1]:.1f} s")

    # Actual sample rate, as flown. If this isn't ~200 Hz, the loop was
    # missing its deadline even if the overrun flag didn't catch it.
    dt = np.diff(data["t_ms"])
    if len(dt):
        print(f"  Tick dt:  {dt.mean():.2f} ms mean, "
              f"{dt.min()}-{dt.max()} ms range")

    # State transitions, with the time each one happened.
    print("\n  State timeline:")
    state = data["state"]
    changes = [0] + list(np.where(np.diff(state))[0] + 1)
    for i in changes:
        s = int(state[i])
        name = STATE_NAMES[s] if s < len(STATE_NAMES) else f"?{s}"
        print(f"    {t[i]:7.2f} s  {name}")

    print(f"\n  Max altitude:     {data['kf_h'].max():8.1f} m")
    print(f"  Max velocity:     {data['kf_v'].max():8.1f} m/s")
    print(f"  Max accel:        {data['lin_accel_z'].max():8.1f} m/s^2 "
          f"({data['lin_accel_z'].max() / GRAVITY:.1f} g)")
    print(f"  Max tilt:         {data['tilt_deg'].max():8.1f} deg")
    print(f"  Max |cmd_y|:      {np.abs(data['cmd_y']).max():8.2f} deg")
    print(f"  Max |cmd_z|:      {np.abs(data['cmd_z']).max():8.2f} deg")

    # Flag counts. The PULSE_Y/PULSE_Z bits are assertions -- guard 1 clamps
    # in degrees before guard 2 ever sees the value, so if these ever appear
    # a calibration constant is wrong and the mixing layer needs a look.
    #
    # Percentages are reported twice. A log is mostly pad time -- a five
    # minute hold around three seconds of boost -- so a whole-log percentage
    # buries what happened during powered flight. 418 saturated ticks is
    # "3% of the log" and also "80% of the burn", and only the second number
    # tells you the controller was pinned for most of the time it mattered.
    print("\n  Flags:")
    flags = data["flags"]
    boost = data["state"] == STATE_BOOST
    n_boost = int(boost.sum())

    if n_boost:
        print(f"    ({n_boost} ticks in BOOST, "
              f"{n_boost * 0.005:.2f} s)")

    any_set = False
    for bit, name, desc in FLAG_BITS:
        hits = (flags & bit) != 0
        n = int(hits.sum())
        if not n:
            continue
        any_set = True
        pct = 100.0 * n / len(flags)
        marker = "  <-- INVESTIGATE" if bit in (0x04, 0x08, 0x20) else ""

        if n_boost:
            n_b = int((hits & boost).sum())
            pct_b = 100.0 * n_b / n_boost
            print(f"    {name:9s} {n:6d} ticks ({pct:5.1f}% of log)  "
                  f"{n_b:5d} in BOOST ({pct_b:5.1f}% of burn)  {desc}{marker}")
        else:
            print(f"    {name:9s} {n:6d} ticks ({pct:5.1f}%)  {desc}{marker}")

    if not any_set:
        print("    none set")


def write_csv(data, path):
    """Write every column, logged and derived, as CSV."""
    names = [n for n in FIELD_NAMES if n != "t_ms"]
    derived = ["t", "tilt_deg", "lin_accel_z", "gyro_mag",
               "wax_g", "way_g", "waz_g"]
    cols = ["t_ms"] + derived + names

    with open(path, "w") as f:
        f.write(",".join(cols) + "\n")
        for i in range(len(data["t"])):
            f.write(",".join(f"{data[c][i]:g}" for c in cols) + "\n")

    print(f"\n  Wrote {path}")


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def shade_states(ax, data):
    """Shade the background by flight state.

    Makes every plot readable at a glance -- you can see immediately whether
    a feature happened during boost, coast, or descent without cross-checking
    timestamps against the state column.
    """
    colors = {0: "#eeeeee", 1: "#e8f0fe", 2: "#ffe8e0",
              3: "#e8f8e8", 4: "#f0e8f8"}
    t = data["t"]
    state = data["state"]

    edges = [0] + list(np.where(np.diff(state))[0] + 1) + [len(t) - 1]
    for a, b in zip(edges, edges[1:]):
        s = int(state[a])
        ax.axvspan(t[a], t[b], color=colors.get(s, "#ffffff"),
                   zorder=0, linewidth=0)


def plot_overview(data, title):
    """Six-panel overview of the whole flight."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    fig.suptitle(title, fontsize=13)
    t = data["t"]

    # --- Altitude ---
    ax = axes[0, 0]
    shade_states(ax, data)
    ax.plot(t, data["kf_h"], lw=1.2)
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude")
    ax.grid(alpha=0.3)

    # --- Velocity ---
    ax = axes[0, 1]
    shade_states(ax, data)
    ax.plot(t, data["kf_v"], lw=1.2, color="tab:orange")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Vertical Velocity")
    ax.grid(alpha=0.3)

    # --- Linear acceleration ---
    ax = axes[1, 0]
    shade_states(ax, data)
    ax.plot(t, data["lin_accel_z"], lw=0.8, color="tab:red")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("Accel (m/s^2)")
    ax.set_title("World-Frame Vertical Acceleration (Gravity Removed)")
    ax.grid(alpha=0.3)

    # --- Tilt ---
    ax = axes[1, 1]
    shade_states(ax, data)
    ax.plot(t, data["tilt_deg"], lw=1.2, color="tab:green")
    ax.set_ylabel("Tilt (deg)")
    ax.set_title("Angle From Vertical")
    ax.grid(alpha=0.3)

    # --- Attitude error ---
    ax = axes[2, 0]
    shade_states(ax, data)
    # err_y/err_z are logged as a rotation vector in radians, whose magnitude
    # is sin(tilt) -- so for the small angles this controller is designed
    # around, converting to degrees gives a number that reads directly as
    # "degrees off vertical" and lines up with the tilt plot beside it.
    ax.plot(t, np.degrees(data["err_y"]), lw=0.9, label="err_y")
    ax.plot(t, np.degrees(data["err_z"]), lw=0.9, label="err_z")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("Error (deg)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Attitude Error About Body Y And Z")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # --- Gimbal commands ---
    ax = axes[2, 1]
    shade_states(ax, data)
    ax.plot(t, data["cmd_y"], lw=0.9, label="cmd_y")
    ax.plot(t, data["cmd_z"], lw=0.9, label="cmd_z")
    # Saturation limits, so you can see at a glance when authority ran out.
    ax.axhline(6.0, color="r", ls="--", lw=0.7, label="MAX_DEFLECT")
    ax.axhline(-6.0, color="r", ls="--", lw=0.7)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("Gimbal (deg)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Commanded Deflection")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    return fig


def plot_detail(data, title):
    """Four-panel detail: raw sensors, servo pulses, quaternion, flags."""
    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    fig.suptitle(title + " -- detail", fontsize=13)
    t = data["t"]

    # --- Body rates ---
    ax = axes[0]
    shade_states(ax, data)
    for name in ("wx", "wy", "wz"):
        ax.plot(t, np.degrees(data[name]), lw=0.7, label=name)
    ax.set_ylabel("Rate (deg/s)")
    ax.set_title("Body Angular Rates (Post Bias Correction)")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(alpha=0.3)

    # --- Raw accelerometer ---
    ax = axes[1]
    shade_states(ax, data)
    for name in ("ax_g", "ay_g", "az_g"):
        ax.plot(t, data[name], lw=0.7, label=name)
    ax.set_ylabel("Accel (g)")
    ax.set_title("Raw Accelerometer, Body Frame")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(alpha=0.3)

    # --- Servo pulses ---
    # This is what physically went to the servos, as opposed to what the
    # control law asked for. If these disagree with cmd_y/cmd_z scaled by
    # the calibration constants, the mixing layer has a problem.
    ax = axes[2]
    shade_states(ax, data)
    ax.plot(t, data["pulse_y"], lw=0.9, label="pulse_y (mount Y)")
    ax.plot(t, data["pulse_z"], lw=0.9, label="pulse_z (mount X)")
    ax.axhline(1575, color="tab:blue", ls=":", lw=0.7)
    ax.axhline(1825, color="tab:orange", ls=":", lw=0.7)
    ax.set_ylabel("Pulse (us)")
    ax.set_title("Servo Pulse Widths (Dotted = Trim)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # --- Flags ---
    # One row per bit, dots where it was set. A sparse scatter is much easier
    # to read here than six stacked line plots.
    ax = axes[3]
    shade_states(ax, data)
    for row, (bit, name, _) in enumerate(FLAG_BITS):
        hits = np.where(data["flags"] & bit)[0]
        if len(hits):
            ax.plot(t[hits], np.full(len(hits), row), "|", ms=8)
    ax.set_yticks(range(len(FLAG_BITS)))
    ax.set_yticklabels([n for _, n, _ in FLAG_BITS], fontsize=8)
    ax.set_ylim(-0.5, len(FLAG_BITS) - 0.5)
    ax.set_xlabel("Time (s)")
    ax.set_title("Log Flags")
    ax.grid(alpha=0.3, axis="x")

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# 3D orientation replay
# ---------------------------------------------------------------------------

def make_rocket_mesh(n_sides=12):
    """Build a simple rocket body in the BODY frame.

    Body +X is the nose axis, matching the firmware's convention, so the model
    is a tube along +X with a cone at the front and fins at the back.

    Returns (verts, faces, colors):
      verts  -- (N,3) array of vertices, body frame
      faces  -- list of index lists, one per polygon
      colors -- one colour per face

    Vertices are stored as one flat array so the whole model can be rotated
    with a single quat_rotate() call per frame; the face lists just index
    into the rotated result.
    """
    verts = []
    faces = []
    colors = []

    def add(x, y, z):
        verts.append([x, y, z])
        return len(verts) - 1

    R = 0.10             # body radius
    tail_x = -0.55       # aft end
    shoulder_x = 0.35    # where the nose cone starts
    tip_x = 0.85         # nose tip

    ang = np.linspace(0, 2 * np.pi, n_sides, endpoint=False)
    cy, cz = R * np.cos(ang), R * np.sin(ang)

    # Rings of vertices at the tail and at the base of the nose cone.
    tail_ring = [add(tail_x, cy[i], cz[i]) for i in range(n_sides)]
    shldr_ring = [add(shoulder_x, cy[i], cz[i]) for i in range(n_sides)]
    tip = add(tip_x, 0.0, 0.0)

    # Body tube: one quad per side.
    for i in range(n_sides):
        j = (i + 1) % n_sides
        faces.append([tail_ring[i], tail_ring[j], shldr_ring[j], shldr_ring[i]])
        colors.append("#d8d8dc")

    # Nose cone: one triangle per side, all meeting at the tip. Deliberately
    # not red -- the body +X arrow is red, and having both the same colour
    # made it hard to tell the model from the axis overlay.
    for i in range(n_sides):
        j = (i + 1) % n_sides
        faces.append([shldr_ring[i], shldr_ring[j], tip])
        colors.append("#e8a33d")

    # Tail cap, so the model isn't hollow when viewed from behind. Dark, so
    # it reads as the nozzle end at a glance.
    faces.append(tail_ring)
    colors.append("#3d3d42")

    # No fins: this vehicle is finless and statically unstable, relying
    # entirely on thrust vectoring for attitude control.

    return np.array(verts), faces, colors


def plot_replay(data, title, decimate=10):
    """Animated 3D replay of the vehicle's attitude.

    Draws a solid rocket body rotated by the logged quaternion, with the
    body-frame axis triad overlaid on top. The triad matters even with the
    model present: the rocket body is rotationally symmetric, so it can't
    show roll on its own -- the green and blue arrows can.

    'decimate' skips frames: at 200 Hz, every 10th sample gives 20 fps, which
    plays back at roughly real time and keeps the animation responsive.

    This is the reason the firmware logs the quaternion rather than Euler
    angles. Euler angles degenerate at nose-vertical -- exactly the attitude
    the rocket spends its whole flight near -- so a replay built from them
    would gimbal-lock right where you most want to see what happened.
    """
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    q = np.column_stack([data["qw"], data["qx"], data["qy"], data["qz"]])
    q = q[::decimate]
    t = data["t"][::decimate]
    state = data["state"][::decimate]
    tilt = data["tilt_deg"][::decimate]

    # Body axes, one row each, rotated into the world frame at every sample.
    n = len(q)
    nose = quat_rotate(q, np.tile([1.0, 0.0, 0.0], (n, 1)))   # body +X
    lat_y = quat_rotate(q, np.tile([0.0, 1.0, 0.0], (n, 1)))  # body +Y
    lat_z = quat_rotate(q, np.tile([0.0, 0.0, 1.0], (n, 1)))  # body +Z

    verts, faces, colors = make_rocket_mesh()

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    def setup_axes():
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("World X")
        ax.set_ylabel("World Y")
        ax.set_zlabel("World Z (up)")
        ax.set_box_aspect([1, 1, 1])

    def draw(i):
        ax.cla()
        setup_axes()

        # World vertical, for reference -- the attitude the controller wants.
        ax.plot([0, 0], [0, 0], [0, 1.15], color="0.6", ls="--", lw=1.0)

        # Rotate every vertex of the model in one shot, then hand the polygons
        # to matplotlib by indexing into the rotated array.
        rot = quat_rotate(np.tile(q[i], (len(verts), 1)), verts)
        polys = [rot[idx] for idx in faces]
        ax.add_collection3d(Poly3DCollection(
            polys, facecolors=colors, edgecolors="#00000030", linewidths=0.4))

        # Axis triad over the body, so the frame stays legible. The nose arrow
        # runs slightly past the tip; the lateral arrows are short.
        ax.quiver(0, 0, 0, *(nose[i] * 1.05), color="tab:red", lw=2.0,
                  arrow_length_ratio=0.12, alpha=0.8)
        ax.quiver(0, 0, 0, *(lat_y[i] * 0.45), color="tab:green", lw=1.4,
                  arrow_length_ratio=0.2, alpha=0.8)
        ax.quiver(0, 0, 0, *(lat_z[i] * 0.45), color="tab:blue", lw=1.4,
                  arrow_length_ratio=0.2, alpha=0.8)

        # Trail of recent nose positions -- makes coning or tumbling obvious.
        lo = max(0, i - 60)
        if i > lo:
            trail = nose[lo:i + 1] * 1.05
            ax.plot(trail[:, 0], trail[:, 1], trail[:, 2],
                    color="tab:red", alpha=0.35, lw=1.0)

        s = int(state[i])
        name = STATE_NAMES[s] if s < len(STATE_NAMES) else f"?{s}"
        ax.set_title(f"{title}\n"
                     f"t = {t[i]:6.2f} s   {name}   tilt = {tilt[i]:5.1f} deg")

    anim = FuncAnimation(fig, draw, frames=n, interval=50, repeat=True)

    # Returning the animation object matters: if it gets garbage collected,
    # matplotlib stops updating and you get a static frame with no error.
    return fig, anim


def save_animation(anim, path, fps=20):
    """Write the replay out as a video or GIF.

    Slow: every frame is a full 3D re-render, so a few hundred frames takes
    a minute or two. Trimming the log first (the default) keeps this
    manageable -- saving an untrimmed log with a long pad hold means
    rendering thousands of frames of a motionless rocket.

    .mp4 needs ffmpeg on PATH. .gif only needs pillow, which comes with
    matplotlib, so it is the safer choice if ffmpeg isn't installed.
    """
    ext = path.rsplit(".", 1)[-1].lower() if "." in path else ""
    writer = "pillow" if ext == "gif" else "ffmpeg"

    print(f"\n  Saving replay to {path} using {writer}...")
    print("  (one 3D render per frame -- this takes a while)")

    try:
        anim.save(path, writer=writer, fps=fps, dpi=100)
    except Exception as e:
        sys.exit(f"\nCould not save the animation: {e}\n"
                 f"  '{writer}' writer failed. An .mp4 needs ffmpeg installed "
                 f"and on PATH;\n"
                 f"  a .gif filename uses pillow instead, which ships with "
                 f"matplotlib.")

    print(f"  Wrote {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="Decode a TVC flight log dump.")
    p.add_argument("logfile", help="PuTTY capture containing the flash dump")
    p.add_argument("--csv", metavar="PATH", help="also write a CSV")
    p.add_argument("--flight", type=int, metavar="N",
                   help="decode only flight N (1-based); default is the last")
    p.add_argument("--replay", action="store_true",
                   help="show the 3D orientation replay")
    p.add_argument("--save-replay", metavar="PATH",
                   help="write the replay to a file (.gif or .mp4); "
                        "implies --replay")
    p.add_argument("--decimate", type=int, default=10,
                   help="replay frame skip (default 10 = ~20 fps)")
    p.add_argument("--no-trim", action="store_true",
                   help="plot the whole log instead of the boost window")
    p.add_argument("--pre-boost", type=float, default=10.0, metavar="SEC",
                   help="seconds of pad time to keep before BOOST (default 10)")
    p.add_argument("--post-descent", type=float, default=10.0, metavar="SEC",
                   help="seconds to keep after DESCENT (default 10)")
    p.add_argument("--no-plots", action="store_true",
                   help="print the summary only")
    args = p.parse_args()

    raw = extract_hex(args.logfile)
    print(f"Extracted {len(raw)} bytes ({len(raw) // PAGE_SIZE} pages)")

    records = parse_records(raw)
    print(f"Parsed {len(records)} records")

    if not records:
        sys.exit("No records found. Was the chip erased?")

    flights = split_flights(records)
    print(f"Found {len(flights)} flight(s):")
    for i, f in enumerate(flights, 1):
        dur = (f[-1][0] - f[0][0]) / 1000.0
        print(f"    {i}: {len(f):6d} records, {dur:6.1f} s")

    if args.flight:
        if not 1 <= args.flight <= len(flights):
            sys.exit(f"No flight {args.flight}; there are {len(flights)}.")
        chosen = [args.flight]
    else:
        # Default to the most recent flight, which is almost always the one
        # you just flew and came here to look at.
        chosen = [len(flights)]

    for idx in chosen:
        flight = flights[idx - 1]
        title = f"Flight {idx} ({len(flight)} records)"
        print(f"\n=== {title} ===")

        data = to_arrays(flight)

        # Summary and CSV always describe the full flight. Only the plots and
        # the replay get trimmed -- flag percentages computed over a trimmed
        # window would be misleading.
        print_summary(data)

        if args.csv:
            path = args.csv
            if len(chosen) > 1:
                path = path.replace(".csv", f"_{idx}.csv")
            write_csv(data, path)

        if not args.no_plots:
            plot_data = data
            if not args.no_trim:
                plot_data, window = trim_flight(
                    data, args.pre_boost, args.post_descent)
                if window is not None:
                    print(f"\n  Plotting {window[0]:.1f}-{window[1]:.1f} s "
                          f"({len(plot_data['t'])} of {len(data['t'])} "
                          f"records). Use --no-trim for everything.")
                else:
                    print("\n  No BOOST in this log -- plotting all of it.")

            plot_overview(plot_data, title)
            plot_detail(plot_data, title)

            if args.replay or args.save_replay:
                # Held in locals so the animation isn't garbage collected
                # before show() or save() runs.
                _fig, _anim = plot_replay(plot_data, title, args.decimate)
                if args.save_replay:
                    save_animation(_anim, args.save_replay,
                                   fps=max(1, 200 // args.decimate))

            plt.show()


if __name__ == "__main__":
    main()