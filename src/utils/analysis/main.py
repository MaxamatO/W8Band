"""
w8band_trajectory.py  —  barbell edition
=========================================
Designed specifically for barbell tracking (squat, deadlift, bench, OHP, etc.).
Sensor assumed at bar center/sleeve.

Key differences from generic IMU trajectory:
  - NO ZUPT: the bar decelerates to zero at rep turnarounds — ZUPT fires there
    and destroys the trace at exactly the most important moments.
  - Rep-boundary anchoring: the bar returns to ~the same rack/floor position
    every rep. We detect turnaround points and linearly detrend position between
    them, which is a much stronger and more physically correct constraint.
  - 15 Hz lowpass (not 10): bar jerk and reversal transients are real signal
    for velocity/power metrics — don't filter them out.
  - SLERP resampling: handles BLE timestamp jitter cleanly.
  - Gravity estimated over a still window before the set, not from a single frame.
  - Per-rep segmentation: each rep plotted + metrics (ROM, peak vel, mean vel).

Columns expected: Qw Qx Qy Qz  Ax Ay Az [mg]  timestamp_ms
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D        # noqa: F401
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.signal import butter, sosfilt, find_peaks
from scipy.interpolate import interp1d

# ── Config ────────────────────────────────────────────────────────────────────
CSV_FILE  = "w8band_data.csv"
IMU_HZ    = 120.0
DT        = 1.0 / IMU_HZ

# Gravity calibration: average over this window at the START of the recording
# while the bar is at rest on the rack / floor. Increase if your recording
# starts with some setup time; decrease if you load the bar immediately.
STILL_START_S = 0.5   # seconds into recording to start calibration
STILL_END_S   = 2.0   # end of calibration window

# Rep detection: turnaround = velocity crosses zero (bar reverses direction).
# PRIMARY_AXIS: 0=X, 1=Y, 2=Z. Set to whichever axis is vertical for your setup.
# For most exercises with sensor on the bar center: Z is up, so use 2.
# If your trace looks flat, try 1 or 0.
PRIMARY_AXIS = 2

# Minimum rep duration in seconds (filters noise peaks from rep detection)
MIN_REP_S = 0.8

# Lowpass cutoff — 15 Hz keeps bar jerk/reversal signal; lower for smoother trace
LP_HZ = 15.0

# ── Load ─────────────────────────────────────────────────────────────────────
df = pd.read_csv(CSV_FILE)
df = df.apply(pd.to_numeric, errors="coerce").dropna()
print(f"Loaded {len(df)} samples.")

quats = df[["Qw","Qx","Qy","Qz"]].values
accel = df[["Ax","Ay","Az"]].values / 1000.0 * 9.81   # mg → m/s²
ts_ms = df["timestamp_ms"].values.astype(float)

# Drop bad quaternions and normalize
mask  = np.linalg.norm(quats, axis=1) > 0.5
quats, accel, ts_ms = quats[mask], accel[mask], ts_ms[mask]
quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)
print(f"After filter: {len(quats)} samples  ({(ts_ms[-1]-ts_ms[0])/1000:.2f} s)")

# ── Resample to uniform grid (fixes BLE jitter) ───────────────────────────────
t_uniform = np.arange(ts_ms[0], ts_ms[-1], DT * 1000.0)
N = len(t_uniform)

accel_rs = np.column_stack([
    interp1d(ts_ms, accel[:, i], kind="linear", fill_value="extrapolate")(t_uniform)
    for i in range(3)
])

rots_raw = R.from_quat(quats[:, [1,2,3,0]])   # scipy convention [x,y,z,w]
slerp    = Slerp(ts_ms, rots_raw)
rots     = slerp(t_uniform)

time_s = (t_uniform - t_uniform[0]) / 1000.0
print(f"Resampled to {N} uniform samples.")

# ── Rotate to world frame ─────────────────────────────────────────────────────
accel_world = rots.apply(accel_rs)

# Gravity calibration over the still window
cal_s = int(STILL_START_S * IMU_HZ)
cal_e = int(STILL_END_S   * IMU_HZ)
if cal_e > N:
    cal_e = N // 4
gravity = np.mean(accel_world[cal_s:cal_e], axis=0)
print(f"Gravity vector: {gravity}  (norm={np.linalg.norm(gravity):.3f} m/s²)")

accel_c = accel_world - gravity

# ── Lowpass (15 Hz — keeps bar dynamics) ─────────────────────────────────────
sos = butter(4, LP_HZ / (IMU_HZ / 2), btype="low", output="sos")
accel_f = np.column_stack([sosfilt(sos, accel_c[:, i]) for i in range(3)])

# ── Integrate (trapezoidal, fixed dt) ────────────────────────────────────────
vel = np.zeros((N, 3))
for i in range(1, N):
    vel[i] = vel[i-1] + (accel_f[i] + accel_f[i-1]) * 0.5 * DT

pos = np.zeros((N, 3))
for i in range(1, N):
    pos[i] = pos[i-1] + (vel[i] + vel[i-1]) * 0.5 * DT

pos_cm = pos * 100.0

# ── Rep detection via velocity zero-crossings on primary axis ─────────────────
v_primary = vel[:, PRIMARY_AXIS]

# Smooth velocity for detection only (not for metrics)
sos2 = butter(2, 3.0 / (IMU_HZ / 2), btype="low", output="sos")
v_smooth = sosfilt(sos2, v_primary)

# Zero-crossings: where sign changes
sign   = np.sign(v_smooth)
crossings = np.where(np.diff(sign) != 0)[0]

# Filter by minimum rep duration
min_rep_samples = int(MIN_REP_S * IMU_HZ)
crossings = [c for i, c in enumerate(crossings)
             if i == 0 or c - crossings[i-1] > min_rep_samples]
crossings = np.array(crossings, dtype=int)

# Add start and end
boundaries = np.concatenate([[0], crossings, [N-1]])
print(f"Detected {len(crossings)} turnarounds → {len(crossings)//2} reps (approx)")

# ── Rep-boundary drift correction ─────────────────────────────────────────────
# At each boundary (turnaround), position should return to roughly the same value.
# We linearly detrend each segment between consecutive boundaries.
# This replaces ZUPT: instead of zeroing velocity (which corrupts motion),
# we correct the integrated position by stretching/compressing it linearly.
pos_corr = pos_cm.copy()

for i in range(len(boundaries) - 1):
    s, e = boundaries[i], boundaries[i+1]
    if e <= s:
        continue
    # Linear drift over this segment
    drift = pos_corr[e] - pos_corr[s]   # how much we drifted from start to end
    t_seg = np.linspace(0, 1, e - s + 1)
    for ax in range(3):
        pos_corr[s:e+1, ax] -= t_seg * drift[ax]

# ── Per-rep segmentation ──────────────────────────────────────────────────────
# Group pairs of boundaries into reps (down + up = 1 rep)
# Each rep = from one bottom/top turnaround back to the next same-direction turnaround.
# Heuristic: pair every 2 consecutive crossings as 1 rep.
rep_segs = []
for i in range(0, len(crossings) - 1, 2):
    s = crossings[i]
    e = crossings[i+1] if i+1 < len(crossings) else N-1
    if e - s > min_rep_samples:
        rep_segs.append((s, e))

print(f"Segmented into {len(rep_segs)} reps.")

speed = np.linalg.norm(vel, axis=1) * 100.0   # cm/s

# ── Plot ──────────────────────────────────────────────────────────────────────
BG = "#0d1117"
C  = ["#ff6b6b", "#51cf66", "#339af0"]
REP_COLORS = plt.cm.plasma(np.linspace(0.15, 0.85, max(len(rep_segs), 1)))

fig = plt.figure(figsize=(18, 11))
fig.patch.set_facecolor(BG)

def style(ax, title, xlabel, ylabel):
    ax.set_facecolor(BG)
    ax.set_title(title, color="white", fontsize=10)
    ax.set_xlabel(xlabel, color="#aaa")
    ax.set_ylabel(ylabel, color="#aaa")
    ax.tick_params(colors="#888")
    ax.spines[["top","right"]].set_visible(False)
    ax.spines[["bottom","left"]].set_color("#444")
    ax.grid(alpha=0.12, color="#555")

# 3-D corrected trajectory, coloured by rep
ax1 = fig.add_subplot(2, 3, (1,4), projection="3d")
ax1.set_facecolor(BG)
for ax in [ax1.xaxis.pane, ax1.yaxis.pane, ax1.zaxis.pane]:
    ax.fill = False; ax.set_edgecolor("#333")

if rep_segs:
    for idx, (s, e) in enumerate(rep_segs):
        ax1.plot(pos_corr[s:e, 0], pos_corr[s:e, 1], pos_corr[s:e, 2],
                 color=REP_COLORS[idx], lw=1.5, alpha=0.9, label=f"Rep {idx+1}")
else:
    ax1.scatter(pos_corr[:,0], pos_corr[:,1], pos_corr[:,2],
                c=time_s, cmap="plasma", s=2, alpha=0.8)

ax1.scatter(*pos_corr[0],  color="#00ff88", s=120, zorder=5, label="Start")
ax1.scatter(*pos_corr[-1], color="#ff4444", s=120, zorder=5, label="End")
ax1.set_xlabel("X cm", color="white"); ax1.set_ylabel("Y cm", color="white")
ax1.set_zlabel("Z cm", color="white")
ax1.set_title("3D trajectory — corrected, per-rep", color="white", fontsize=12)
ax1.tick_params(colors="white")
ax1.legend(facecolor="#1a1a2e", labelcolor="white", fontsize=8, loc="upper left")

# Primary axis position vs time with rep markers
ax2 = fig.add_subplot(2, 3, 2)
style(ax2, f"Axis {'XYZ'[PRIMARY_AXIS]} position (primary, corrected)", "s", "cm")
ax2.plot(time_s, pos_corr[:, PRIMARY_AXIS], color=C[PRIMARY_AXIS], lw=1.3)
for idx, (s, e) in enumerate(rep_segs):
    ax2.axvspan(time_s[s], time_s[e], alpha=0.12, color=REP_COLORS[idx])
for c in crossings:
    ax2.axvline(time_s[c], color="#555", lw=0.6, ls="--")

# Speed vs time
ax3 = fig.add_subplot(2, 3, 3)
style(ax3, "Bar speed", "s", "cm/s")
ax3.plot(time_s, speed, color="#f59f00", lw=1.3)
for idx, (s, e) in enumerate(rep_segs):
    ax3.axvspan(time_s[s], time_s[e], alpha=0.12, color=REP_COLORS[idx])

# Per-rep ROM bar chart
ax4 = fig.add_subplot(2, 3, 5)
style(ax4, "ROM per rep", "rep", "cm")
if rep_segs:
    roms = [np.ptp(pos_corr[s:e, PRIMARY_AXIS]) for s, e in rep_segs]
    bars = ax4.bar(range(1, len(roms)+1), roms, color=[REP_COLORS[i] for i in range(len(roms))])
    ax4.set_xticks(range(1, len(roms)+1))
    for bar, rom in zip(bars, roms):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                 f"{rom:.1f}", ha="center", va="bottom", color="white", fontsize=8)

# Per-rep mean concentric velocity
ax5 = fig.add_subplot(2, 3, 6)
style(ax5, "Mean bar speed per rep", "rep", "cm/s")
if rep_segs:
    mean_vels = [np.mean(speed[s:e]) for s, e in rep_segs]
    bars2 = ax5.bar(range(1, len(mean_vels)+1), mean_vels,
                    color=[REP_COLORS[i] for i in range(len(mean_vels))])
    ax5.set_xticks(range(1, len(mean_vels)+1))
    for bar, mv in zip(bars2, mean_vels):
        ax5.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                 f"{mv:.1f}", ha="center", va="bottom", color="white", fontsize=8)

plt.suptitle("W8Band – Barbell trajectory", color="white", fontsize=14, y=1.01)
plt.tight_layout()
plt.savefig("w8band_trajectory.png", dpi=150, bbox_inches="tight", facecolor=BG)
print("Saved: w8band_trajectory.png")
plt.show()

# ── Console summary ───────────────────────────────────────────────────────────
print("\n── Per-rep summary ──")
for idx, (s, e) in enumerate(rep_segs):
    rom      = np.ptp(pos_corr[s:e, PRIMARY_AXIS])
    peak_v   = np.max(speed[s:e])
    mean_v   = np.mean(speed[s:e])
    duration = time_s[e] - time_s[s]
    print(f"  Rep {idx+1:2d}: {duration:.2f}s  ROM {rom:.1f}cm  "
          f"peak {peak_v:.1f}cm/s  mean {mean_v:.1f}cm/s")