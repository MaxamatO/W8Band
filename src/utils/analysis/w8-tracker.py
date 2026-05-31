"""
w8band_trajectory_2d.py  —  bench press edition
================================================
Tor ruchu 2D (X vs Z) z ZUPT w dolnym punkcie ruchu.

Zmiany względem oryginału:
  - Tylko 2D: oś pozioma X, oś pionowa Z
  - ZUPT: detekcja dolnego punktu przez okno wariancji + prędkość ≈ 0
    → reset prędkości do zera → drift nie kumuluje się między fazami
  - Okno nagrania 2-3s na rep + 2s kalibracji → zwiększono czułość detekcji
  - Boundary correction pozostaje jako dodatkowe zabezpieczenie po ZUPT
  - Uproszczone wykresy: tor 2D + prędkość + ROM per rep + mean vel per rep

Kolumny CSV: Qw Qx Qy Qz  Ax Ay Az [mg]  timestamp_ms
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.signal import butter, sosfilt
from scipy.interpolate import interp1d

# ── Konfiguracja ──────────────────────────────────────────────────────────────
CSV_FILE      = "w8band_data.csv"
IMU_HZ        = 120.0
DT            = 1.0 / IMU_HZ

# Kalibracja grawitacji — sztanga leży nieruchomo na początku nagrania
STILL_START_S = 0.3
STILL_END_S   = 1.8   # krótsze okno bo nagranie krótsze

# Oś pionowa (Z=2), pozioma do 2D (X=0)
PRIMARY_AXIS  = 2
HORIZ_AXIS    = 0

# Filtr dolnoprzepustowy
LP_HZ         = 15.0

# Minimalna długość repu w sekundach
MIN_REP_S     = 0.6   # niższy próg bo krótsze ruchy przy bench

# ── ZUPT — parametry detekcji spoczynku ──────────────────────────────────────
# Okno w próbkach do liczenia wariancji przyspieszenia
ZUPT_WINDOW   = 18    # ~0.15s przy 120 Hz
# Próg wariancji normy |a| — gdy mniejszy: czujnik stoi
ZUPT_VAR_THR  = 0.003  # (m/s²)² — dostrój jeśli ZUPT odpala za często/rzadko
# Próg prędkości pionowej — gdy |v_z| < tego: kandydat na spoczynek
ZUPT_VEL_THR  = 0.08   # m/s
# Ile kolejnych próbek musi spełniać warunki zanim ZUPT odpali
ZUPT_CONFIRM  = 10

# ── Wczytanie danych ─────────────────────────────────────────────────────────
df = pd.read_csv(CSV_FILE)
df = df.apply(pd.to_numeric, errors="coerce").dropna()
print(f"Wczytano {len(df)} próbek.")

quats = df[["Qw","Qx","Qy","Qz"]].values
accel = df[["Ax","Ay","Az"]].values / 1000.0 * 9.81   # mg → m/s²
ts_ms = df["timestamp_ms"].values.astype(float)

# Normalizacja kwaternionów
mask  = np.linalg.norm(quats, axis=1) > 0.5
quats, accel, ts_ms = quats[mask], accel[mask], ts_ms[mask]
quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)
print(f"Po filtrze: {len(quats)} próbek  ({(ts_ms[-1]-ts_ms[0])/1000:.2f} s)")

# ── Resample do równomiernej siatki (eliminuje jitter BLE) ────────────────────
t_uniform = np.arange(ts_ms[0], ts_ms[-1], DT * 1000.0)
N = len(t_uniform)

accel_rs = np.column_stack([
    interp1d(ts_ms, accel[:, i], kind="linear",
             fill_value="extrapolate")(t_uniform)
    for i in range(3)
])

rots_raw = R.from_quat(quats[:, [1,2,3,0]])   # scipy: [x,y,z,w]
slerp    = Slerp(ts_ms, rots_raw)
rots     = slerp(t_uniform)
time_s   = (t_uniform - t_uniform[0]) / 1000.0
print(f"Resample: {N} próbek.")

# ── Obrót do układu świata ────────────────────────────────────────────────────
accel_world = rots.apply(accel_rs)

# Kalibracja grawitacji na oknie spoczynku
cal_s = max(0, int(STILL_START_S * IMU_HZ))
cal_e = min(N, int(STILL_END_S   * IMU_HZ))
gravity = np.mean(accel_world[cal_s:cal_e], axis=0)
print(f"Grawitacja: {gravity}  (norma={np.linalg.norm(gravity):.3f} m/s²)")

accel_c = accel_world - gravity

# ── Filtr dolnoprzepustowy 15 Hz ─────────────────────────────────────────────
sos     = butter(4, LP_HZ / (IMU_HZ / 2), btype="low", output="sos")
accel_f = np.column_stack([sosfilt(sos, accel_c[:, i]) for i in range(3)])

# ── Całkowanie trapezami + ZUPT ───────────────────────────────────────────────
vel       = np.zeros((N, 3))
pos       = np.zeros((N, 3))
zupt_mask = np.zeros(N, dtype=bool)   # gdzie ZUPT odpalił — do wizualizacji

# Norma przyspieszenia do detekcji spoczynku
a_norm = np.linalg.norm(accel_f, axis=1)

# Licznik kolejnych próbek spełniających warunki ZUPT
zupt_counter = 0

for i in range(1, N):
    # Całkowanie trapezami
    vel[i] = vel[i-1] + (accel_f[i] + accel_f[i-1]) * 0.5 * DT
    pos[i] = pos[i-1] + (vel[i]     + vel[i-1])     * 0.5 * DT

    # ── Detekcja spoczynku (dolny punkt bench press) ──────────────────────────
    # Warunek 1: wariancja normy |a| w ostatnim oknie jest mała
    if i >= ZUPT_WINDOW:
        var_a = float(np.var(a_norm[i - ZUPT_WINDOW:i]))
    else:
        var_a = 999.0

    # Warunek 2: prędkość pionowa bliska zeru
    v_vert = abs(vel[i, PRIMARY_AXIS])

    if var_a < ZUPT_VAR_THR and v_vert < ZUPT_VEL_THR:
        zupt_counter += 1
    else:
        zupt_counter = 0

    # Gdy oba warunki spełnione przez ZUPT_CONFIRM próbek z rzędu → reset
    if zupt_counter >= ZUPT_CONFIRM:
        vel[i] = 0.0          # zeruj prędkość — wiesz że stoi
        zupt_mask[i] = True

pos_cm = pos * 100.0

# ── Boundary correction (dodatkowe zabezpieczenie po ZUPT) ───────────────────
# Detekcja turnaroundów przez zero-crossings wygładzonej prędkości
v_primary = vel[:, PRIMARY_AXIS]
sos2      = butter(2, 3.0 / (IMU_HZ / 2), btype="low", output="sos")
v_smooth  = sosfilt(sos2, v_primary)

sign      = np.sign(v_smooth)
crossings = np.where(np.diff(sign) != 0)[0]

min_rep_samples = int(MIN_REP_S * IMU_HZ)
filtered_c = []
for i, c in enumerate(crossings):
    if i == 0 or c - filtered_c[-1] > min_rep_samples:
        filtered_c.append(c)
crossings = np.array(filtered_c, dtype=int)

boundaries = np.concatenate([[0], crossings, [N-1]])
print(f"Turnaroundy: {len(crossings)}  →  ~{len(crossings)//2} repów")

pos_corr = pos_cm.copy()
for i in range(len(boundaries) - 1):
    s, e = boundaries[i], boundaries[i+1]
    if e <= s:
        continue
    drift  = pos_corr[e] - pos_corr[s]
    t_seg  = np.linspace(0, 1, e - s + 1)
    for ax in range(3):
        pos_corr[s:e+1, ax] -= t_seg * drift[ax]

# ── Segmentacja repów ─────────────────────────────────────────────────────────
rep_segs = []
for i in range(0, len(crossings) - 1, 2):
    s = crossings[i]
    e = crossings[i+1] if i+1 < len(crossings) else N-1
    if e - s > min_rep_samples:
        rep_segs.append((s, e))

print(f"Repów: {len(rep_segs)}")

speed = np.linalg.norm(vel, axis=1) * 100.0  # cm/s

# ── Metryki per rep ───────────────────────────────────────────────────────────
reps_data = []
for s, e in rep_segs:
    rom      = float(np.ptp(pos_corr[s:e, PRIMARY_AXIS]))
    peak_v   = float(np.max(speed[s:e]))
    mean_v   = float(np.mean(speed[s:e]))
    duration = float(time_s[e] - time_s[s])
    reps_data.append(dict(rom=rom, peak_v=peak_v, mean_v=mean_v, dur=duration))

# ── Styl ──────────────────────────────────────────────────────────────────────
BG         = "#0d1117"
GRID_COLOR = "#1e2430"
TICK_COLOR = "#4a5568"
LABEL_C    = "#8892a0"
TEXT_C     = "#e2e8f0"
ACCENT     = "#00e5ff"
ACCENT2    = "#ff3d71"
GREEN      = "#00ff9d"
YELLOW     = "#ffd60a"

N_REPS     = max(len(rep_segs), 1)
REP_COLORS = plt.cm.plasma(np.linspace(0.15, 0.85, N_REPS))

plt.rcParams.update({
    "font.family":      "monospace",
    "axes.facecolor":   BG,
    "figure.facecolor": BG,
    "text.color":       TEXT_C,
    "axes.labelcolor":  LABEL_C,
    "xtick.color":      TICK_COLOR,
    "ytick.color":      TICK_COLOR,
    "axes.edgecolor":   GRID_COLOR,
    "grid.color":       GRID_COLOR,
    "grid.alpha":       0.5,
})

fig = plt.figure(figsize=(16, 10))
fig.patch.set_facecolor(BG)

gs = gridspec.GridSpec(
    2, 3,
    figure=fig,
    left=0.06, right=0.97,
    top=0.92,  bottom=0.08,
    wspace=0.32, hspace=0.38
)

def style_ax(ax, title, xlabel, ylabel):
    ax.set_title(title, color=TEXT_C, fontsize=10, pad=8)
    ax.set_xlabel(xlabel, color=LABEL_C, fontsize=9)
    ax.set_ylabel(ylabel, color=LABEL_C, fontsize=9)
    ax.spines[["top","right"]].set_visible(False)
    ax.spines[["bottom","left"]].set_color(GRID_COLOR)
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=8)

# ── 1. Tor ruchu 2D  (zajmuje lewą kolumnę obie wiersze) ─────────────────────
ax_traj = fig.add_subplot(gs[:, 0])
style_ax(ax_traj, "Tor ruchu 2D", "Poziomo X  (cm)", "Pionowo Z  (cm)")

if rep_segs:
    for idx, (s, e) in enumerate(rep_segs):
        col = REP_COLORS[idx]
        ax_traj.plot(
            pos_corr[s:e, HORIZ_AXIS],
            pos_corr[s:e, PRIMARY_AXIS],
            color=col, lw=2.2, alpha=0.92,
            label=f"Rep {idx+1}"
        )
        # punkt startowy repu
        ax_traj.scatter(
            pos_corr[s, HORIZ_AXIS], pos_corr[s, PRIMARY_AXIS],
            color=col, s=55, zorder=6, edgecolors="white", linewidths=0.5
        )
else:
    ax_traj.plot(
        pos_corr[:, HORIZ_AXIS], pos_corr[:, PRIMARY_AXIS],
        color=ACCENT, lw=1.5, alpha=0.8
    )

# Zaznacz gdzie ZUPT odpalił
zupt_idx = np.where(zupt_mask)[0]
if len(zupt_idx):
    ax_traj.scatter(
        pos_corr[zupt_idx, HORIZ_AXIS],
        pos_corr[zupt_idx, PRIMARY_AXIS],
        color=YELLOW, s=18, zorder=7, alpha=0.6, label="ZUPT"
    )

ax_traj.scatter(
    pos_corr[0, HORIZ_AXIS],  pos_corr[0, PRIMARY_AXIS],
    color=GREEN,  s=100, zorder=8, label="Start"
)
ax_traj.scatter(
    pos_corr[-1, HORIZ_AXIS], pos_corr[-1, PRIMARY_AXIS],
    color=ACCENT2, s=100, zorder=8, label="Koniec"
)

ax_traj.legend(
    facecolor="#12151c", edgecolor=GRID_COLOR,
    labelcolor=TEXT_C, fontsize=8, loc="best", framealpha=0.9
)
ax_traj.set_aspect("equal", adjustable="datalim")

# ── 2. Pozycja Z w czasie ─────────────────────────────────────────────────────
ax_pos = fig.add_subplot(gs[0, 1])
style_ax(ax_pos, "Pozycja pionowa Z", "Czas (s)", "cm")

ax_pos.plot(time_s, pos_corr[:, PRIMARY_AXIS],
            color=ACCENT, lw=1.4, alpha=0.9)

for idx, (s, e) in enumerate(rep_segs):
    ax_pos.axvspan(time_s[s], time_s[e],
                   alpha=0.10, color=REP_COLORS[idx])

# linie ZUPT
for c in crossings:
    ax_pos.axvline(time_s[c], color=TICK_COLOR, lw=0.5, ls="--", alpha=0.6)

# zaznacz momenty ZUPT
if len(zupt_idx):
    ax_pos.scatter(
        time_s[zupt_idx],
        pos_corr[zupt_idx, PRIMARY_AXIS],
        color=YELLOW, s=14, zorder=5, alpha=0.7
    )

# ── 3. Prędkość w czasie ──────────────────────────────────────────────────────
ax_vel = fig.add_subplot(gs[1, 1])
style_ax(ax_vel, "Prędkość", "Czas (s)", "cm/s")

ax_vel.plot(time_s, speed, color=TICK_COLOR, lw=0.7, alpha=0.5)

for idx, (s, e) in enumerate(rep_segs):
    col = REP_COLORS[idx]
    ax_vel.fill_between(time_s[s:e], speed[s:e],
                        alpha=0.20, color=col)
    ax_vel.plot(time_s[s:e], speed[s:e],
                color=col, lw=1.8, alpha=0.9)

# ── 4. ROM per rep ────────────────────────────────────────────────────────────
ax_rom = fig.add_subplot(gs[0, 2])
style_ax(ax_rom, "ROM per rep", "Rep", "cm")

if reps_data:
    roms  = [r["rom"] for r in reps_data]
    x_rep = range(1, len(roms) + 1)
    bars  = ax_rom.bar(x_rep, roms,
                       color=[REP_COLORS[i] for i in range(len(roms))],
                       width=0.55, zorder=3)
    ax_rom.set_xticks(list(x_rep))
    for bar, rom in zip(bars, roms):
        ax_rom.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.4,
            f"{rom:.1f}", ha="center", va="bottom",
            color=TEXT_C, fontsize=8
        )

# ── 5. Średnia prędkość per rep ───────────────────────────────────────────────
ax_mv = fig.add_subplot(gs[1, 2])
style_ax(ax_mv, "Średnia prędkość per rep", "Rep", "cm/s")

if reps_data:
    mean_vels = [r["mean_v"] for r in reps_data]
    bars2     = ax_mv.bar(x_rep, mean_vels,
                          color=[REP_COLORS[i] for i in range(len(mean_vels))],
                          width=0.55, zorder=3)
    ax_mv.set_xticks(list(x_rep))
    for bar, mv in zip(bars2, mean_vels):
        ax_mv.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.4,
            f"{mv:.1f}", ha="center", va="bottom",
            color=TEXT_C, fontsize=8
        )

# ── Tytuł ─────────────────────────────────────────────────────────────────────
n_zupt = int(zupt_mask.sum())
fig.suptitle(
    f"W8Band — Bench Press  ·  {len(rep_segs)} repów  ·  ZUPT: {n_zupt} resetów",
    color=TEXT_C, fontsize=13, y=0.97
)

plt.savefig("w8band_2d.png", dpi=150, bbox_inches="tight", facecolor=BG)
print("Zapisano: w8band_2d.png")
plt.show()

# ── Podsumowanie w konsoli ────────────────────────────────────────────────────
print("\n── Podsumowanie per rep ──")
for idx, r in enumerate(reps_data):
    print(f"  Rep {idx+1:2d}: {r['dur']:.2f}s  "
          f"ROM {r['rom']:.1f}cm  "
          f"Vmax {r['peak_v']:.1f}cm/s  "
          f"Vśr {r['mean_v']:.1f}cm/s")

print(f"\nZUPT odpalił {n_zupt}× "
      f"(żółte punkty na wykresach = momenty resetu prędkości)")