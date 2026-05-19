"""
w8band_trajectory.py
====================
Wczytuje CSV z w8band_receiver.py i rysuje trajektorię 3D + diagnostykę.

CSV columns: Qw, Qx, Qy, Qz, Ax, Ay, Az [mg], timestamp_ms
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, sosfilt

CSV_FILE = "w8band_data.csv"

# ============================================================
# 1. WCZYTANIE
# ============================================================
df = pd.read_csv(CSV_FILE)
df = df.apply(pd.to_numeric, errors="coerce").dropna()
print(f"Wczytano {len(df)} próbek.")

# Quaterniony (Qw Qx Qy Qz) – kolejność jak w pliku
quats = df[["Qw", "Qx", "Qy", "Qz"]].values

# Filtr: odrzuć wiersze z kwaternionem bliskim zeru
norm_q = np.linalg.norm(quats, axis=1)
mask   = norm_q > 0.5
quats  = quats[mask]
df     = df[mask].reset_index(drop=True)

# Akceleracja [mg] → [m/s²]
accel_local = df[["Ax", "Ay", "Az"]].values / 1000.0 * 9.81

# Timestampy
ts_ms = df["timestamp_ms"].values.astype(float)

# Rzeczywisty dt między próbkami (nie zakładamy stałego 120 Hz)
dt_arr = np.diff(ts_ms) / 1000.0          # sekundy między sąsiednimi próbkami
dt_arr = np.clip(dt_arr, 1e-4, 0.1)       # odrzuć patologiczne wartości

N = len(df)
print(f"Po filtracji: {N} próbek")
print(f"Czas nagrania: {(ts_ms[-1]-ts_ms[0])/1000:.2f} s")
print(f"Średni dt: {dt_arr.mean()*1000:.2f} ms | "
      f"Hz ≈ {1/dt_arr.mean():.1f}")

# ============================================================
# 2. OBRÓT DO UKŁADU GLOBALNEGO
# ============================================================
# scipy: from_quat oczekuje [x, y, z, w]
rots = R.from_quat(quats[:, [1, 2, 3, 0]])   # Qx Qy Qz Qw

# Wyrównaj do układu startowego
START = int(0.3 * (1 / dt_arr.mean()))   # pierwszych ~0.3 s to spokój po kliknięciu
START = max(1, min(START, N // 10))
q_ref_inv  = rots[START].inv()
rots_align = q_ref_inv * rots

accel_global = rots_align.apply(accel_local)

# ============================================================
# 3. KOMPENSACJA GRAWITACJI
# ============================================================
END = min(START + int(0.5 * (1 / dt_arr.mean())), N // 5)
gravity = np.mean(accel_global[START:END], axis=0)
print(f"Wektor grawitacji (układ globalny): {gravity}")

accel_clean = accel_global - gravity

# ============================================================
# 4. WYGŁADZANIE (butterworth, nie rolling – zachowuje fazy)
# ============================================================
FREQ_HZ  = 1 / dt_arr.mean()
CUTOFF   = 10.0   # Hz – górna częstotliwość dla ruchu ręki
sos = butter(4, CUTOFF / (FREQ_HZ / 2), btype="low", output="sos")
for ax in range(3):
    accel_clean[:, ax] = sosfilt(sos, accel_clean[:, ax])

# ============================================================
# 5. ZUPT (Zero-Velocity Update)
# ============================================================
mag = np.linalg.norm(accel_clean, axis=1)

# Automatyczny próg: mediana + 1.5× IQR
q25, q75 = np.percentile(mag, [25, 75])
THRESHOLD = np.percentile(mag, 25) + 0.5 * (q75 - q25)
THRESHOLD = max(THRESHOLD, 0.3)   # minimum 0.3 m/s²
print(f"ZUPT próg: {THRESHOLD:.3f} m/s²")

WINDOW = max(3, int(0.12 * FREQ_HZ))   # 120 ms
stationary = np.zeros(N, dtype=bool)
for i in range(N):
    s = max(0, i - WINDOW // 2)
    e = min(N,  i + WINDOW // 2)
    if np.all(mag[s:e] < THRESHOLD):
        stationary[i] = True

# ============================================================
# 6. CAŁKOWANIE (trapezowe, dt z timestampów)
# ============================================================
velocity = np.zeros((N, 3))
for i in range(1, N):
    dt = dt_arr[i - 1]
    if stationary[i]:
        velocity[i] = np.zeros(3)
    else:
        velocity[i] = velocity[i-1] + \
                      (accel_clean[i] + accel_clean[i-1]) / 2.0 * dt

position = np.zeros((N, 3))
for i in range(1, N):
    dt = dt_arr[i - 1]
    position[i] = position[i-1] + \
                  (velocity[i] + velocity[i-1]) / 2.0 * dt

pos_cm = position * 100.0   # metry → cm

# ============================================================
# 7. RYSOWANIE
# ============================================================
time_s = (ts_ms - ts_ms[0]) / 1000.0

fig = plt.figure(figsize=(18, 10))
fig.patch.set_facecolor("#0d1117")

# ── Subplot 1: Trajektoria 3D ────────────────────────────
ax1 = fig.add_subplot(2, 3, (1, 4), projection="3d")
ax1.set_facecolor("#0d1117")
sc = ax1.scatter(pos_cm[:, 0], pos_cm[:, 1], pos_cm[:, 2],
                 c=time_s, cmap="plasma", s=2, alpha=0.7)
ax1.scatter(*pos_cm[0],  color="#00ff88", s=80, zorder=5, label="Start")
ax1.scatter(*pos_cm[-1], color="#ff4444", s=80, zorder=5, label="Koniec")
ax1.set_xlabel("X (cm)", color="white")
ax1.set_ylabel("Y (cm)", color="white")
ax1.set_zlabel("Z (cm)", color="white")
ax1.set_title("Trajektoria 3D", color="white", fontsize=13)
ax1.tick_params(colors="white")
ax1.xaxis.pane.fill = False
ax1.yaxis.pane.fill = False
ax1.zaxis.pane.fill = False
for pane in [ax1.xaxis.pane, ax1.yaxis.pane, ax1.zaxis.pane]:
    pane.set_edgecolor("#333")
ax1.legend(facecolor="#1a1a2e", labelcolor="white")
plt.colorbar(sc, ax=ax1, label="Czas (s)", shrink=0.6)

# ── Subplot 2: X, Y, Z vs czas ──────────────────────────
ax2 = fig.add_subplot(2, 3, 2)
ax2.set_facecolor("#0d1117")
for i, (col, label) in enumerate(zip(range(3), ["X", "Y", "Z"])):
    ax2.plot(time_s, pos_cm[:, col], label=label,
             color=["#ff6b6b","#51cf66","#339af0"][i], linewidth=1.2)
ax2.set_title("Pozycja vs czas", color="white")
ax2.set_xlabel("Czas (s)", color="white")
ax2.set_ylabel("cm", color="white")
ax2.tick_params(colors="white")
ax2.legend(facecolor="#1a1a2e", labelcolor="white")
ax2.spines[["bottom","left"]].set_color("#444")
ax2.spines[["top","right"]].set_visible(False)
ax2.grid(alpha=0.15)

# ── Subplot 3: Prędkość ─────────────────────────────────
ax3 = fig.add_subplot(2, 3, 3)
ax3.set_facecolor("#0d1117")
speed = np.linalg.norm(velocity, axis=1) * 100   # cm/s
ax3.plot(time_s, speed, color="#f59f00", linewidth=1.2, label="Prędkość (cm/s)")
ax3.fill_between(time_s, 0, speed * stationary, color="#00ff88", alpha=0.3, label="ZUPT aktywny")
ax3.set_title("Prędkość", color="white")
ax3.set_xlabel("Czas (s)", color="white")
ax3.set_ylabel("cm/s", color="white")
ax3.tick_params(colors="white")
ax3.legend(facecolor="#1a1a2e", labelcolor="white")
ax3.spines[["bottom","left"]].set_color("#444")
ax3.spines[["top","right"]].set_visible(False)
ax3.grid(alpha=0.15)

# ── Subplot 4: ZUPT diagnostyka ─────────────────────────
ax4 = fig.add_subplot(2, 3, 5)
ax4.set_facecolor("#0d1117")
ax4.plot(time_s, mag, color="#f03e3e", linewidth=1.0, label="‖a_clean‖")
ax4.axhline(THRESHOLD, color="white", linestyle="--", linewidth=0.8, label=f"Próg ZUPT ({THRESHOLD:.2f} m/s²)")
ax4.fill_between(time_s, 0, mag.max(), where=stationary,
                 color="#00ff88", alpha=0.2, label="ZUPT ON")
ax4.set_title("Diagnostyka ZUPT", color="white")
ax4.set_xlabel("Czas (s)", color="white")
ax4.set_ylabel("m/s²", color="white")
ax4.tick_params(colors="white")
ax4.legend(facecolor="#1a1a2e", labelcolor="white", fontsize=8)
ax4.spines[["bottom","left"]].set_color("#444")
ax4.spines[["top","right"]].set_visible(False)
ax4.grid(alpha=0.15)

# ── Subplot 5: dt między próbkami (kontrola jakości BLE) ─
ax5 = fig.add_subplot(2, 3, 6)
ax5.set_facecolor("#0d1117")
ax5.plot(time_s[1:], dt_arr * 1000, color="#cc5de8", linewidth=0.8)
ax5.axhline(1000 / 120, color="white", linestyle="--", linewidth=0.8,
            label=f"Idealny dt ({1000/120:.1f} ms)")
ax5.set_title("dt między próbkami (jakość BLE)", color="white")
ax5.set_xlabel("Czas (s)", color="white")
ax5.set_ylabel("ms", color="white")
ax5.tick_params(colors="white")
ax5.legend(facecolor="#1a1a2e", labelcolor="white", fontsize=8)
ax5.spines[["bottom","left"]].set_color("#444")
ax5.spines[["top","right"]].set_visible(False)
ax5.grid(alpha=0.15)

plt.suptitle("W8Band – Analiza trajektorii", color="white", fontsize=15, y=1.01)
plt.tight_layout()
plt.savefig("w8band_trajectory.png", dpi=150, bbox_inches="tight",
            facecolor="#0d1117")
print("Zapisano: w8band_trajectory.png")
plt.show()