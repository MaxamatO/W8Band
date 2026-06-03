"""
w8band_ekf.py — single-rep barbell trajectory (bench press)
============================================================
Pipeline:
  1. Wczytanie CSV + normalizacja kwaternionów
  2. Resample SLERP → równomierna siatka 120 Hz
  3. Rotacja akcelerometru do układu świata (kwaternion SFLP)
  4. Kalibracja grawitacji w oknie REST przed ruchem
  5. EKF (state: pos, vel, accel_bias 3+3+3=9D)
       - propagacja: model stałego przyspieszenia + bias walk
       - update:     pomiar przyspieszenia w świecie
  6. Adaptive ZUPT: state machine (REST→ECCENTRIC→TURNAROUND→CONCENTRIC→REST)
       - kryteria: var(|a|), |v_z|, jerk, gyro-proxy (|dq/dt|), hystereza
       - ZUPT update: vel=0, zeruje P(vel) w EKF
  7. RTS smoother: backward pass po EKF → wygładzona trajektoria
  8. Wykres 2D: X vs Z, pojedynczy rep, fazy kolorem

Kolumny CSV: Qw Qx Qy Qz  Ax Ay Az [mg]  Seq  timestamp_ms
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import interp1d
from scipy.signal import butter, sosfilt

# ══════════════════════════════════════════════════════════════════════════════
#  KONFIGURACJA
# ══════════════════════════════════════════════════════════════════════════════
CSV_FILE     = "w8band_data.csv"
IMU_HZ       = 120.0
DT           = 1.0 / IMU_HZ

# Oś pionowa i pozioma
VERT         = 2   # Z
HORIZ        = 0   # X

# Okno kalibracji grawitacji [s] — czujnik MUSI leżeć nieruchomo
STILL_S      = (0.2, 1.8)

# ── EKF ───────────────────────────────────────────────────────────────────────
# Szum procesu
Q_POS        = 1e-6   # (m²/s⁴) — pozycja
Q_VEL        = 1e-4   # (m²/s⁴) — prędkość
Q_BIAS       = 1e-7   # (m²/s⁵) — random walk biasu akc.

# Szum pomiaru akcelerometru (światowy)
R_ACCEL      = 0.05   # (m/s²)²  — dostrój: większy = bardziej ufa propagacji

# ── ZUPT ──────────────────────────────────────────────────────────────────────
# Progi wejścia w REST (wszystkie muszą być spełnione)
ZUPT_VAR_THR    = 8e-3   # wariancja |a_world| (m/s²)²
ZUPT_VEL_THR    = 0.06   # |v_z| m/s
ZUPT_JERK_THR   = 1.5    # |jerk| m/s³
ZUPT_GYROP_THR  = 0.08   # proxy gyro: |dq/dt| (rad/s, aproksymacja)
ZUPT_CONFIRM    = 12     # próbek z rzędu zanim zmieni stan
ZUPT_HYSTER     = 8      # próbek z rzędu niespełnionych zanim wyjdzie z REST

# Szum pomiaru ZUPT — bardzo mały bo jesteśmy PEWNI że v=0
R_ZUPT          = 1e-6   # (m/s)²

# Okno jitter-smoothing przed detekcją faz
JERK_WIN        = 5      # próbek

# ── Wykres ────────────────────────────────────────────────────────────────────
BG           = "#0A0C10"
COL_REST     = "#4a5568"
COL_ECC      = "#00b4d8"
COL_TURN     = "#ffd60a"
COL_CON      = "#ff3d71"
TRAJ_LW      = 2.4

# ══════════════════════════════════════════════════════════════════════════════
#  1. WCZYTANIE I NORMALIZACJA
# ══════════════════════════════════════════════════════════════════════════════
df   = pd.read_csv(CSV_FILE)
df   = df.apply(pd.to_numeric, errors="coerce").dropna()
print(f"Wczytano {len(df)} próbek.")

quats = df[["Qw","Qx","Qy","Qz"]].values
accel = df[["Ax","Ay","Az"]].values / 1000.0 * 9.81   # mg → m/s²
ts_ms = df["timestamp_ms"].values.astype(float)

norms = np.linalg.norm(quats, axis=1)
mask  = norms > 0.5
quats, accel, ts_ms = quats[mask], accel[mask], ts_ms[mask]
quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)
print(f"Po filtrze: {len(quats)} próbek  ({(ts_ms[-1]-ts_ms[0])/1e3:.2f} s)")

# ══════════════════════════════════════════════════════════════════════════════
#  2. RESAMPLE → równomierna siatka 120 Hz  (SLERP + liniowy akc.)
# ══════════════════════════════════════════════════════════════════════════════
t_uni = np.arange(ts_ms[0], ts_ms[-1], DT * 1000.0)
N     = len(t_uni)

accel_rs = np.column_stack([
    interp1d(ts_ms, accel[:, i], kind="linear",
             fill_value="extrapolate")(t_uni)
    for i in range(3)
])

rots_raw = R.from_quat(quats[:, [1, 2, 3, 0]])   # scipy [x,y,z,w]
slerp    = Slerp(ts_ms, rots_raw)
rots     = slerp(t_uni)
time_s   = (t_uni - t_uni[0]) / 1e3
print(f"Resample: {N} próbek.")

# Proxy gyro: kątowa prędkość ≈ |Δq| / dt  (aproksymacja liniowa na sferwach)
# Używamy jako jeden z sygnałów ZUPT — bez logowania surowego gyro
quats_rs   = rots.as_quat()                    # [x,y,z,w] × N
dq         = np.diff(quats_rs, axis=0)
gyro_proxy = np.linalg.norm(dq, axis=1) / DT  # [rad/s aproks.]
gyro_proxy = np.append(gyro_proxy, gyro_proxy[-1])

# ══════════════════════════════════════════════════════════════════════════════
#  3. ROTACJA DO UKŁADU ŚWIATA
# ══════════════════════════════════════════════════════════════════════════════
a_world = rots.apply(accel_rs)   # [m/s²], układ świata

# ══════════════════════════════════════════════════════════════════════════════
#  4. KALIBRACJA GRAWITACJI
# ══════════════════════════════════════════════════════════════════════════════
cs = max(0, int(STILL_S[0] * IMU_HZ))
ce = min(N, int(STILL_S[1] * IMU_HZ))
g_vec = np.mean(a_world[cs:ce], axis=0)
print(f"Grawitacja: {g_vec}  (norma={np.linalg.norm(g_vec):.4f} m/s²)")

a_comp = a_world - g_vec   # przyspieszenie ruchu

# ══════════════════════════════════════════════════════════════════════════════
#  5. EKF
# ══════════════════════════════════════════════════════════════════════════════
# Stan: x = [px, py, pz, vx, vy, vz, bx, by, bz]  (9D)
# b = bias akcelerometru (random walk)
# Pomiar: a_world_measured = a_true + b  →  innowacja = a_comp - b_est

# Macierze
F = np.eye(9)                     # model stałego przyspieszenia
F[0:3, 3:6] = np.eye(3) * DT     # p += v*dt
F[3:6, 6:9] = -np.eye(3) * DT    # v -= b*dt  (bias koryguje prędkość)

Q = np.zeros((9, 9))
Q[0:3, 0:3] = np.eye(3) * Q_POS * DT**2
Q[3:6, 3:6] = np.eye(3) * Q_VEL * DT
Q[6:9, 6:9] = np.eye(3) * Q_BIAS * DT

H_accel = np.zeros((3, 9))       # pomiar: obserwujemy bias przez akcelerometr
H_accel[0:3, 6:9] = np.eye(3)   # innowacja = a_comp[k] - b_est → H*x = b

R_a = np.eye(3) * R_ACCEL
R_z = np.eye(3) * R_ZUPT         # dla ZUPT: pomiar v=0

# Inicjalizacja
x  = np.zeros(9)
P  = np.diag([1e-4]*3 + [1e-4]*3 + [1e-6]*3)

# Bufory do RTS smoothera
xs = np.zeros((N, 9))
Ps = np.zeros((N, 9, 9))
Fs_buf = np.zeros((N, 9, 9))   # F jest stałe, ale zapisujemy dla RTS

# ── State machine ─────────────────────────────────────────────────────────────
# Stany: 0=REST, 1=ECCENTRIC, 2=TURNAROUND, 3=CONCENTRIC
REST, ECCENTRIC, TURNAROUND, CONCENTRIC = 0, 1, 2, 3
STATE_NAMES = ["REST", "ECCENTRIC", "TURNAROUND", "CONCENTRIC"]

phase      = np.zeros(N, dtype=int)
state      = REST
conf_cnt   = 0    # licznik potwierdzenia wejścia w REST
exit_cnt   = 0    # licznik wyjścia z REST (hystereza)

# Norma przyspieszenia + jerk do detekcji faz
a_norm_all = np.linalg.norm(a_comp, axis=1)

sos_lp = butter(2, 10.0 / (IMU_HZ / 2), btype="low", output="sos")
a_norm_smooth = sosfilt(sos_lp, a_norm_all)

jerk_raw = np.abs(np.gradient(a_norm_smooth, DT))
jerk_raw = sosfilt(sos_lp, jerk_raw)

# EKF forward pass
zupt_mask = np.zeros(N, dtype=bool)

for k in range(N):
    # ── Propagacja ───────────────────────────────────────────────────────────
    x = F @ x
    x[3:6] += a_comp[k] * DT      # v += a_compensated * dt
    P = F @ P @ F.T + Q

    # ── Pomiar akcelerometru (update biasu) ───────────────────────────────────
    # innowacja: obserwujemy a_comp[k] ≈ -bias (ruch już w v, bias powoli dryfuje)
    # Uwaga: używamy słabego update — głównie do estymacji biasu, nie pozycji
    z_a = a_comp[k]                         # obserwacja
    y   = z_a - H_accel @ x                 # innowacja
    S   = H_accel @ P @ H_accel.T + R_a
    K   = P @ H_accel.T @ np.linalg.inv(S)
    x   = x + K @ y
    P   = (np.eye(9) - K @ H_accel) @ P

    # ── Detekcja faz — State Machine ─────────────────────────────────────────
    v_z_est  = x[4]                         # prędkość pionowa z EKF
    var_a    = float(np.var(
        a_norm_all[max(0, k-12):k+1]))
    jerk_k   = jerk_raw[k]
    gyrop_k  = gyro_proxy[k]

    rest_cond = (
        var_a   < ZUPT_VAR_THR   and
        abs(v_z_est) < ZUPT_VEL_THR  and
        jerk_k  < ZUPT_JERK_THR  and
        gyrop_k < ZUPT_GYROP_THR
    )

    if state == REST:
        if rest_cond:
            exit_cnt = 0
        else:
            exit_cnt += 1
            if exit_cnt >= ZUPT_HYSTER:
                # Wychodzi z REST — kierunek v_z decyduje o fazie
                state    = ECCENTRIC if v_z_est <= 0 else CONCENTRIC
                conf_cnt = 0
                exit_cnt = 0

    elif state == ECCENTRIC:
        # Turnaround: prędkość pionowa zmienia znak na dodatni
        if v_z_est > 0.03:
            state = TURNAROUND
        # Jeśli znowu nieruchomy — wróć do REST
        elif rest_cond:
            conf_cnt += 1
            if conf_cnt >= ZUPT_CONFIRM:
                state    = REST
                conf_cnt = 0
        else:
            conf_cnt = 0

    elif state == TURNAROUND:
        # Z TURNAROUND wychodzimy gdy prędkość jest wyraźnie dodatnia
        if v_z_est > 0.08:
            state = CONCENTRIC
        elif rest_cond:
            conf_cnt += 1
            if conf_cnt >= ZUPT_CONFIRM:
                state    = REST
                conf_cnt = 0
        else:
            conf_cnt = 0

    elif state == CONCENTRIC:
        # Wróć do REST gdy nieruchomy (koniec repu)
        if rest_cond:
            conf_cnt += 1
            if conf_cnt >= ZUPT_CONFIRM:
                state    = REST
                conf_cnt = 0
        else:
            conf_cnt = 0

    phase[k] = state

    # ── ZUPT: EKF update z pomiarem v=0 ──────────────────────────────────────
    # Odpala w REST oraz w TURNAROUND (sztanga chwilę stoi)
    do_zupt = (state == REST) or (
        state == TURNAROUND and
        abs(v_z_est) < ZUPT_VEL_THR * 1.5 and
        var_a < ZUPT_VAR_THR * 3.0
    )

    if do_zupt:
        H_v = np.zeros((3, 9))
        H_v[0:3, 3:6] = np.eye(3)     # obserwujemy v
        z_v = np.zeros(3)              # pomiar: v = 0
        y_v = z_v - H_v @ x
        S_v = H_v @ P @ H_v.T + R_z
        K_v = P @ H_v.T @ np.linalg.inv(S_v)
        x   = x + K_v @ y_v
        P   = (np.eye(9) - K_v @ H_v) @ P
        zupt_mask[k] = True

    # Zapisz stan do RTS
    xs[k]    = x.copy()
    Ps[k]    = P.copy()
    Fs_buf[k] = F.copy()

print(f"EKF forward pass done. ZUPT odpalił {zupt_mask.sum()} razy.")

# ══════════════════════════════════════════════════════════════════════════════
#  7. RTS SMOOTHER (backward pass)
# ══════════════════════════════════════════════════════════════════════════════
xs_s = xs.copy()
Ps_s = Ps.copy()

for k in range(N - 2, -1, -1):
    F_k  = Fs_buf[k]
    P_k  = Ps[k]
    P_k1 = Ps[k + 1]

    # Predicted covariance na krok k+1
    P_pred = F_k @ P_k @ F_k.T + Q

    # Smoother gain
    G = P_k @ F_k.T @ np.linalg.inv(P_pred)

    xs_s[k] = xs[k] + G @ (xs_s[k + 1] - F_k @ xs[k])
    Ps_s[k] = P_k   + G @ (Ps_s[k + 1] - P_pred) @ G.T

print("RTS smoother done.")

# ══════════════════════════════════════════════════════════════════════════════
#  Wyciągnięcie trajektorii
# ══════════════════════════════════════════════════════════════════════════════
pos_m   = xs_s[:, 0:3]          # [m]
pos_cm  = pos_m * 100.0          # [cm]

# ══════════════════════════════════════════════════════════════════════════════
#  8. WYKRES 2D — tylko tor ruchu, fazy kolorem
# ══════════════════════════════════════════════════════════════════════════════
PHASE_COL = {
    REST:       COL_REST,
    ECCENTRIC:  COL_ECC,
    TURNAROUND: COL_TURN,
    CONCENTRIC: COL_CON,
}

plt.rcParams.update({
    "font.family":      "monospace",
    "axes.facecolor":   BG,
    "figure.facecolor": BG,
})

fig, ax = plt.subplots(figsize=(7, 9))
fig.patch.set_facecolor(BG)
ax.set_facecolor(BG)
ax.tick_params(colors="#4a5568", labelsize=9)
for sp in ax.spines.values():
    sp.set_color("#1e2430")
ax.set_xlabel("Poziomo X  (cm)", color="#8892a0", fontsize=10)
ax.set_ylabel("Pionowo Z  (cm)",  color="#8892a0", fontsize=10)
ax.set_title("W8Band — Tor ruchu sztangi", color="#e2e8f0",
             fontsize=13, pad=14)
ax.grid(True, linewidth=0.35, color="#1e2430", alpha=0.8)

# Rysuj segmenty kolorem fazy
for k in range(1, N):
    col = PHASE_COL[phase[k]]
    ax.plot(
        [pos_cm[k-1, HORIZ], pos_cm[k, HORIZ]],
        [pos_cm[k-1, VERT],  pos_cm[k, VERT]],
        color=col, lw=TRAJ_LW, solid_capstyle="round"
    )

# Start / koniec
ax.scatter(pos_cm[0,  HORIZ], pos_cm[0,  VERT],
           color="#00ff9d", s=120, zorder=10,
           edgecolors="white", linewidths=0.8, label="Start")
ax.scatter(pos_cm[-1, HORIZ], pos_cm[-1, VERT],
           color="#ff3d71", s=120, zorder=10,
           edgecolors="white", linewidths=0.8, label="Koniec")

# Legenda faz
patches = [
    mpatches.Patch(color=COL_REST,     label="REST"),
    mpatches.Patch(color=COL_ECC,      label="ECCENTRIC (opad)"),
    mpatches.Patch(color=COL_TURN,     label="TURNAROUND"),
    mpatches.Patch(color=COL_CON,      label="CONCENTRIC (push)"),
    mpatches.Patch(color="#00ff9d",    label="Start"),
    mpatches.Patch(color="#ff3d71",    label="Koniec"),
]
ax.legend(handles=patches,
          facecolor="#12151c", edgecolor="#1e2430",
          labelcolor="#e2e8f0", fontsize=8,
          loc="best", framealpha=0.92)

ax.set_aspect("equal", adjustable="datalim")
plt.tight_layout(pad=1.6)

out = "w8band_trajectory_ekf.png"
plt.savefig(out, dpi=160, bbox_inches="tight", facecolor=BG)
print(f"Zapisano: {out}")
plt.show()

# ══════════════════════════════════════════════════════════════════════════════
#  Konsola — metryki jednego repu
# ══════════════════════════════════════════════════════════════════════════════
vel_cm = xs_s[:, 3:6] * 100.0   # cm/s
speed  = np.linalg.norm(vel_cm, axis=1)

# Faza CONCENTRIC = push
con_mask = phase == CONCENTRIC
ecc_mask = phase == ECCENTRIC

rom_z    = float(np.ptp(pos_cm[:, VERT]))
rom_x    = float(np.ptp(pos_cm[:, HORIZ]))

print("\n── Metryki jednego repu ──")
print(f"  ROM pionowo (Z):       {rom_z:.1f} cm")
print(f"  ROM poziomo (X):       {rom_x:.1f} cm")
if con_mask.any():
    print(f"  Vmax concentric:       {speed[con_mask].max():.1f} cm/s")
    print(f"  Vmean concentric:      {speed[con_mask].mean():.1f} cm/s")
if ecc_mask.any():
    print(f"  Vmax eccentric:        {speed[ecc_mask].max():.1f} cm/s")
print(f"  Bias akc. estymowany:  {xs_s[-1, 6:9]*1000} mg")
print(f"  ZUPT resetów:          {zupt_mask.sum()}")

# Czas trwania faz
for sid, sname in enumerate(STATE_NAMES):
    dur = float((phase == sid).sum()) * DT
    print(f"  Czas {sname:<12s}:  {dur:.2f} s")