"""
W8Band trajectory analysis.

This version intentionally does not run an extra Python EKF. The LSM6DSV16X
already provides fused quaternions, so Python treats them as the orientation
source, rotates accelerometer samples into the global frame, removes gravity,
and reconstructs trajectory with zero-velocity references plus linear
velocity detrending.

Input CSV columns:
    Qw Qx Qy Qz Ax Ay Az Seq timestamp_ms

Output:
    w8band_trajectory_ekf.png

The output filename is kept for compatibility with w8band_ui.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/w8band_matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from scipy.signal import butter, find_peaks, sosfiltfilt
from scipy.spatial.transform import Rotation as R, Slerp

BASE_DIR = Path(__file__).resolve().parent
CSV_FILE = BASE_DIR / "w8band_data2.csv"
OUT_IMAGE = BASE_DIR / "w8band_trajectory_ekf.png"

IMU_HZ = 120.0
DT = 1.0 / IMU_HZ
G = 9.80665

HORIZ = 0
MIN_SEGMENT_S = 0.30
TURNAROUND_WINDOW_S = 0.10
REST_WINDOW_S = 0.25
QUIET_PERCENTILE = 25.0

BG = "#0A0C10"
COL_REST = "#4a5568"
COL_ECC = "#00b4d8"
COL_TURN = "#ffd60a"
COL_CON = "#ff3d71"
COL_START = "#00ff9d"
COL_END = "#ff3d71"

REST, ECCENTRIC, TURNAROUND, CONCENTRIC = 0, 1, 2, 3
STATE_NAMES = ["REST", "ECCENTRIC", "TURNAROUND", "CONCENTRIC"]
PHASE_COLORS = {
    REST: COL_REST,
    ECCENTRIC: COL_ECC,
    TURNAROUND: COL_TURN,
    CONCENTRIC: COL_CON,
}


@dataclass
class Samples:
    seq: np.ndarray
    time_s: np.ndarray
    quats_wxyz: np.ndarray
    accel_mps2: np.ndarray


@dataclass
class Analysis:
    time_s: np.ndarray
    accel_world: np.ndarray
    accel_linear: np.ndarray
    gravity: np.ndarray
    velocity: np.ndarray
    position: np.ndarray
    phase: np.ndarray
    references: list[int]
    vertical_axis: int
    vertical_sign: float
    rotation_mode: str
    lost_samples: int


def rolling_mean(x: np.ndarray, window: int) -> np.ndarray:
    window = max(1, int(window))
    if window <= 1:
        return x.copy()
    kernel = np.ones(window, dtype=float) / float(window)
    return np.convolve(x, kernel, mode="same")


def rolling_var(x: np.ndarray, window: int) -> np.ndarray:
    mean = rolling_mean(x, window)
    mean2 = rolling_mean(x * x, window)
    return np.maximum(0.0, mean2 - mean * mean)


def lowpass(data: np.ndarray, cutoff_hz: float, order: int = 2) -> np.ndarray:
    if len(data) < 12:
        return data.copy()
    sos = butter(order, cutoff_hz / (IMU_HZ / 2.0), btype="low", output="sos")
    try:
        return sosfiltfilt(sos, data, axis=0)
    except ValueError:
        return data.copy()


def load_samples(csv_path: Path) -> Samples:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    required = ["Qw", "Qx", "Qy", "Qz", "Ax", "Ay", "Az", "Seq"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"CSV is missing columns: {', '.join(missing)}")

    df = df.apply(pd.to_numeric, errors="coerce").dropna(subset=required)
    df = df.sort_values("Seq").drop_duplicates("Seq", keep="first")

    quats = df[["Qw", "Qx", "Qy", "Qz"]].to_numpy(dtype=float)
    accel = df[["Ax", "Ay", "Az"]].to_numpy(dtype=float) / 1000.0 * G
    seq = df["Seq"].to_numpy(dtype=float)

    norms = np.linalg.norm(quats, axis=1)
    mask = norms > 0.5
    quats = quats[mask]
    accel = accel[mask]
    seq = seq[mask]
    norms = norms[mask]

    if len(seq) < 20:
        raise ValueError(f"Too few valid samples: {len(seq)}")

    quats = quats / norms[:, None]
    time_s = (seq - seq[0]) / IMU_HZ

    print(f"Loaded {len(seq)} valid samples from {csv_path.name}.")
    print(f"Seq range: {int(seq[0])}..{int(seq[-1])}, duration {time_s[-1]:.2f} s.")

    return Samples(seq=seq, time_s=time_s, quats_wxyz=quats, accel_mps2=accel)


def resample(samples: Samples) -> tuple[np.ndarray, np.ndarray, R]:
    t = samples.time_s
    t_uniform = np.arange(t[0], t[-1] + DT * 0.5, DT)

    accel_rs = np.column_stack(
        [
            interp1d(t, samples.accel_mps2[:, axis], kind="linear", fill_value="extrapolate")(
                t_uniform
            )
            for axis in range(3)
        ]
    )

    # Scipy expects [x, y, z, w]. Firmware CSV stores [w, x, y, z].
    rots_raw = R.from_quat(samples.quats_wxyz[:, [1, 2, 3, 0]])
    rots = Slerp(t, rots_raw)(t_uniform)

    print(f"Resampled to {len(t_uniform)} samples at {IMU_HZ:.0f} Hz.")
    return t_uniform, accel_rs, rots


def estimate_quiet_mask(accel_world: np.ndarray, gyro_proxy: np.ndarray) -> np.ndarray:
    a_norm = np.linalg.norm(accel_world, axis=1)
    window = max(5, int(REST_WINDOW_S * IMU_HZ))
    score = rolling_var(a_norm, window) + 0.03 * rolling_mean(gyro_proxy, window)
    threshold = np.percentile(score, QUIET_PERCENTILE)
    mask = score <= threshold

    near_g = np.abs(a_norm - G) < 1.5
    if np.count_nonzero(mask & near_g) >= max(10, len(accel_world) // 20):
        mask = mask & near_g

    return mask


def gravity_from_quiet(accel_world: np.ndarray, quiet_mask: np.ndarray) -> np.ndarray:
    if np.count_nonzero(quiet_mask) < 10:
        quiet_mask = np.ones(len(accel_world), dtype=bool)

    median_g = np.median(accel_world[quiet_mask], axis=0)
    norm = float(np.linalg.norm(median_g))
    if norm < 1e-6:
        raise ValueError("Could not estimate gravity vector.")

    # Use measured direction but nominal magnitude. This avoids injecting a
    # scale/bias error from a quiet window into every integrated sample.
    return median_g / norm * G


def choose_rotation_mode(accel_rs: np.ndarray, rots: R) -> tuple[np.ndarray, np.ndarray, str]:
    quats_xyzw = rots.as_quat()
    dq = np.diff(quats_xyzw, axis=0, prepend=quats_xyzw[[0]])
    gyro_proxy = np.linalg.norm(dq, axis=1) / DT

    candidates = {
        "quat.apply(accel)": rots.apply(accel_rs),
        "quat.inv().apply(accel)": rots.inv().apply(accel_rs),
    }

    best_name = ""
    best_accel = None
    best_gravity = None
    best_score = float("inf")

    for name, accel_world in candidates.items():
        quiet = estimate_quiet_mask(accel_world, gyro_proxy)
        gravity = gravity_from_quiet(accel_world, quiet)
        residual = accel_world[quiet] - gravity
        score = float(np.median(np.linalg.norm(residual, axis=1)))
        print(f"Rotation candidate {name}: quiet residual median {score:.4f} m/s^2")
        if score < best_score:
            best_score = score
            best_name = name
            best_accel = accel_world
            best_gravity = gravity

    assert best_accel is not None and best_gravity is not None
    print(f"Using rotation mode: {best_name}")
    print(f"Gravity vector: {best_gravity} norm={np.linalg.norm(best_gravity):.4f} m/s^2")
    return best_accel, best_gravity, best_name


def trapezoid_integrate(values: np.ndarray) -> np.ndarray:
    out = np.zeros_like(values)
    for i in range(1, len(values)):
        out[i] = out[i - 1] + 0.5 * (values[i] + values[i - 1]) * DT
    return out


def raw_integrate(accel_linear: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    vel = trapezoid_integrate(accel_linear)
    pos = trapezoid_integrate(vel)
    return vel, pos


def reference_points(accel_linear: np.ndarray, vertical_axis: int) -> list[int]:
    vel_raw, pos_raw = raw_integrate(accel_linear)
    vz = lowpass(vel_raw[:, vertical_axis], cutoff_hz=4.0)
    z = lowpass(pos_raw[:, vertical_axis], cutoff_hz=4.0)
    n = len(z)

    min_distance = max(1, int(MIN_SEGMENT_S * IMU_HZ))
    min_prom = max(0.02, 0.20 * float(np.ptp(z)))

    peaks, peak_props = find_peaks(z, distance=min_distance, prominence=min_prom)
    troughs, trough_props = find_peaks(-z, distance=min_distance, prominence=min_prom)
    extrema = np.concatenate([peaks, troughs]).astype(int)

    # Prefer extrema near vertical velocity zero crossings.
    zeroish = np.abs(vz) < max(0.03, np.percentile(np.abs(vz), 25.0))
    filtered: list[int] = []
    edge = int(0.20 * IMU_HZ)
    for idx in sorted(extrema.tolist()):
        if idx <= edge or idx >= n - edge:
            continue
        lo = max(0, idx - int(0.08 * IMU_HZ))
        hi = min(n, idx + int(0.08 * IMU_HZ) + 1)
        if np.any(zeroish[lo:hi]):
            filtered.append(idx)

    refs = [0]
    for idx in filtered:
        if idx - refs[-1] >= min_distance:
            refs.append(idx)
    if n - 1 - refs[-1] >= min_distance:
        refs.append(n - 1)
    elif refs[-1] != n - 1:
        refs[-1] = n - 1

    if len(refs) < 2:
        refs = [0, n - 1]

    print(f"Reference points: {refs}")
    return refs


def integrate_with_velocity_detrend(accel_linear: np.ndarray, refs: list[int]) -> tuple[np.ndarray, np.ndarray]:
    n = len(accel_linear)
    velocity = np.zeros((n, 3))
    position = np.zeros((n, 3))

    for start, end in zip(refs[:-1], refs[1:]):
        if end <= start:
            continue

        seg_accel = accel_linear[start : end + 1]
        seg_vel = trapezoid_integrate(seg_accel)
        duration = (end - start) * DT
        if duration > 0:
            drift = seg_vel[-1] / duration
            tau = np.arange(len(seg_vel), dtype=float)[:, None] * DT
            seg_vel = seg_vel - tau * drift

        seg_pos_delta = trapezoid_integrate(seg_vel)
        velocity[start : end + 1] = seg_vel
        position[start : end + 1] = position[start] + seg_pos_delta
        if end + 1 < n:
            position[end + 1] = position[end]

    return velocity, position


def classify_phases(
    velocity: np.ndarray,
    position: np.ndarray,
    refs: list[int],
    vertical_axis: int,
    vertical_sign: float,
) -> np.ndarray:
    n = len(position)
    phase = np.full(n, REST, dtype=int)
    vz = velocity[:, vertical_axis] * vertical_sign

    for start, end in zip(refs[:-1], refs[1:]):
        if end <= start:
            continue
        mid = (start + end) // 2
        mean_v = float(np.mean(vz[start + 1 : end + 1]))
        phase_id = CONCENTRIC if mean_v > 0 else ECCENTRIC
        phase[start:end + 1] = phase_id

        turn_half = int(TURNAROUND_WINDOW_S * IMU_HZ)
        lo = max(start, end - turn_half)
        hi = min(n, end + turn_half + 1)
        phase[lo:hi] = TURNAROUND

    phase[: max(1, int(0.08 * IMU_HZ))] = REST
    phase[max(0, n - int(0.08 * IMU_HZ)) :] = REST
    return phase


def analyze(csv_path: Path = CSV_FILE) -> Analysis:
    samples = load_samples(csv_path)
    expected = int(samples.seq[-1] - samples.seq[0] + 1)
    lost_samples = max(0, expected - len(samples.seq))
    if lost_samples:
        print(f"Warning: sequence gaps suggest {lost_samples} lost sample(s).")

    time_s, accel_rs, rots = resample(samples)
    accel_world, gravity, rotation_mode = choose_rotation_mode(accel_rs, rots)
    accel_linear = accel_world - gravity
    accel_linear = lowpass(accel_linear, cutoff_hz=10.0)

    vertical_axis = int(np.argmax(np.abs(gravity)))
    vertical_sign = 1.0 if gravity[vertical_axis] >= 0 else -1.0
    print(f"Vertical axis inferred from gravity: {['X', 'Y', 'Z'][vertical_axis]}")

    refs = reference_points(accel_linear, vertical_axis)
    velocity, position = integrate_with_velocity_detrend(accel_linear, refs)
    phase = classify_phases(velocity, position, refs, vertical_axis, vertical_sign)

    return Analysis(
        time_s=time_s,
        accel_world=accel_world,
        accel_linear=accel_linear,
        gravity=gravity,
        velocity=velocity,
        position=position,
        phase=phase,
        references=refs,
        vertical_axis=vertical_axis,
        vertical_sign=vertical_sign,
        rotation_mode=rotation_mode,
        lost_samples=lost_samples,
    )


def plot_analysis(result: Analysis, out_path: Path = OUT_IMAGE) -> None:
    pos_cm = result.position * 100.0
    vel_cm_s = result.velocity * 100.0
    speed = np.linalg.norm(vel_cm_s, axis=1)
    vert = result.vertical_axis
    horiz = HORIZ if HORIZ != vert else (1 if vert != 1 else 0)

    plt.rcParams.update(
        {
            "font.family": "monospace",
            "axes.facecolor": BG,
            "figure.facecolor": BG,
        }
    )

    fig, (ax_traj, ax_speed) = plt.subplots(
        1,
        2,
        figsize=(12, 7),
        gridspec_kw={"width_ratios": [1.05, 0.95]},
    )
    fig.patch.set_facecolor(BG)

    for ax in (ax_traj, ax_speed):
        ax.set_facecolor(BG)
        ax.tick_params(colors="#8892a0", labelsize=8)
        for sp in ax.spines.values():
            sp.set_color("#1e2430")
        ax.grid(True, linewidth=0.35, color="#1e2430", alpha=0.8)

    ax_traj.set_title("W8Band - trajectory", color="#e2e8f0", fontsize=13, pad=12)
    ax_traj.set_xlabel(f"Horizontal {['X', 'Y', 'Z'][horiz]} (cm)", color="#8892a0")
    ax_traj.set_ylabel(f"Vertical {['X', 'Y', 'Z'][vert]} (cm)", color="#8892a0")

    for k in range(1, len(pos_cm)):
        color = PHASE_COLORS[int(result.phase[k])]
        ax_traj.plot(
            [pos_cm[k - 1, horiz], pos_cm[k, horiz]],
            [pos_cm[k - 1, vert], pos_cm[k, vert]],
            color=color,
            lw=2.0,
            solid_capstyle="round",
        )

    ax_traj.scatter(
        pos_cm[0, horiz],
        pos_cm[0, vert],
        color=COL_START,
        s=90,
        zorder=10,
        edgecolors="white",
        linewidths=0.7,
    )
    ax_traj.scatter(
        pos_cm[-1, horiz],
        pos_cm[-1, vert],
        color=COL_END,
        s=90,
        zorder=10,
        edgecolors="white",
        linewidths=0.7,
    )

    for idx in result.references[1:-1]:
        ax_traj.scatter(
            pos_cm[idx, horiz],
            pos_cm[idx, vert],
            color=COL_TURN,
            s=55,
            zorder=9,
            edgecolors="#0A0C10",
            linewidths=0.8,
        )

    ax_traj.set_aspect("equal", adjustable="datalim")

    ax_speed.set_title("Speed and reference points", color="#e2e8f0", fontsize=13, pad=12)
    ax_speed.set_xlabel("Time (s)", color="#8892a0")
    ax_speed.set_ylabel("Speed (cm/s)", color="#8892a0")
    ax_speed.plot(result.time_s, speed, color="#e2e8f0", lw=1.2)
    for idx in result.references:
        ax_speed.axvline(result.time_s[idx], color=COL_TURN, lw=0.9, alpha=0.65)

    patches = [
        mpatches.Patch(color=COL_REST, label="REST"),
        mpatches.Patch(color=COL_ECC, label="ECCENTRIC"),
        mpatches.Patch(color=COL_TURN, label="TURNAROUND/ref"),
        mpatches.Patch(color=COL_CON, label="CONCENTRIC"),
        mpatches.Patch(color=COL_START, label="Start"),
        mpatches.Patch(color=COL_END, label="End"),
    ]
    ax_traj.legend(
        handles=patches,
        facecolor="#12151c",
        edgecolor="#1e2430",
        labelcolor="#e2e8f0",
        fontsize=8,
        loc="best",
        framealpha=0.92,
    )

    plt.tight_layout(pad=1.6)
    fig.savefig(out_path, dpi=160, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"Saved: {out_path}")


def print_metrics(result: Analysis) -> None:
    pos_cm = result.position * 100.0
    vel_cm_s = result.velocity * 100.0
    speed = np.linalg.norm(vel_cm_s, axis=1)
    vert = result.vertical_axis
    horiz = HORIZ if HORIZ != vert else (1 if vert != 1 else 0)

    rom_v = float(np.ptp(pos_cm[:, vert]))
    rom_h = float(np.ptp(pos_cm[:, horiz]))
    duration = float(result.time_s[-1] - result.time_s[0])

    print("\nTrajectory metrics")
    print(f"  Duration:              {duration:.2f} s")
    print(f"  Vertical ROM:          {rom_v:.1f} cm")
    print(f"  Horizontal ROM:        {rom_h:.1f} cm")
    print(f"  Peak speed:            {np.max(speed):.1f} cm/s")
    print(f"  Mean speed:            {np.mean(speed):.1f} cm/s")
    print(f"  Reference points:      {len(result.references)}")
    print(f"  Lost samples by Seq:   {result.lost_samples}")
    print(f"  Rotation mode:         {result.rotation_mode}")

    for sid, name in enumerate(STATE_NAMES):
        phase_time = np.count_nonzero(result.phase == sid) * DT
        print(f"  Time {name:<12s}: {phase_time:.2f} s")


def main() -> None:
    result = analyze(CSV_FILE)
    plot_analysis(result, OUT_IMAGE)
    print_metrics(result)


if __name__ == "__main__":
    main()
