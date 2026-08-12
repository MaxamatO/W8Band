"""
Offline pipeline do analizy pojedynczego powtorzenia zarejestrowanego
przez W8Band. Wklej surowy CSV z DumpRawDataToSerial() i uruchom.

Kolumny CSV: seq,timestamp_ms,qw,qx,qy,qz,ax,ay,az,gvx,gvy,gvz
(qw..qz to surowe int16 Q1.14; ax,ay,az i gvx,gvy,gvz to surowe int16 mg)

Wyjscie: pelna trajektoria 3D w ukladzie swiata (X,Y,Z), zwizualizowana jako
dwa rzuty 2D (X-Z i Y-Z) - jeden z nich powinien pokazac naturalny luk ruchu
sztangi (plaszczyzna strzalkowa), drugi powinien zostac plaski blisko zera
(drift boczny). Bez magnetometru nie da sie z gory okreslic ktory to ktory -
ocen to wizualnie po ksztalcie.
"""

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Konfiguracja - dopasuj do swoich zmierzonych wartosci
# ---------------------------------------------------------------------------
CSV_PATH = "dataset.csv"
ACCEL_BIAS_MG = np.array([0.0, 0.0, 0.0])  # wklej m_AccelBias z sesji nagrania
Q14_SCALE = 16384.0
MG_TO_MS2 = 9.80665 / 1000.0
LOWPASS_CUTOFF_HZ = 10.0
SAMPLE_RATE_HZ = 120.0


def rotate_body_to_world(qw, qx, qy, qz, vx, vy, vz):
    """Ten sam wzor co RotateBodyToWorld w C++ - wektoryzowany po calej serii."""
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx

    ccx = qy * cz - qz * cy
    ccy = qz * cx - qx * cz
    ccz = qx * cy - qy * cx

    out_x = vx + 2.0 * qw * cx + 2.0 * ccx
    out_y = vy + 2.0 * qw * cy + 2.0 * ccy
    out_z = vz + 2.0 * qw * cz + 2.0 * ccz
    return out_x, out_y, out_z


def linear_detrend(v, i_start, i_end):
    """Wymusza v[i_start]=0 i v[i_end]=0 przez odjecie liniowej rampy.
    Dziala na widoku v[i_start:i_end+1] w miejscu, nic nie zwraca."""
    n = i_end - i_start
    if n <= 0:
        return
    segment = v[i_start:i_end + 1]
    ramp = np.linspace(segment[0], segment[-1], n + 1)
    v[i_start:i_end + 1] = segment - ramp


def integrate_trapezoidal(values, dt, start_value=0.0):
    out = np.empty(len(values))
    out[0] = start_value
    for i in range(1, len(values)):
        out[i] = out[i - 1] + (values[i - 1] + values[i]) * 0.5 * dt[i - 1]
    return out


def process_axis(world_accel_mg, dt, turnaround_idx, cutoff_hz, fs_hz):
    """Filtr -> calkowanie -> korekcja ZUPT dwuodcinkowa (start/turnaround/
    koniec) -> pozycja. Ten sam pipeline dla kazdej z trzech osi swiata."""
    b, a = butter(2, cutoff_hz, fs=fs_hz, btype="low")
    filt_mg = filtfilt(b, a, world_accel_mg)
    accel_ms2 = filt_mg * MG_TO_MS2

    v_raw = integrate_trapezoidal(accel_ms2, dt)
    v_corrected = v_raw.copy()
    linear_detrend(v_corrected, 0, turnaround_idx)
    linear_detrend(v_corrected, turnaround_idx, len(v_corrected) - 1)

    position = integrate_trapezoidal(v_corrected, dt)
    return v_corrected, position


# ---------------------------------------------------------------------------
# 1. Wczytanie i dekodowanie
# ---------------------------------------------------------------------------
df = pd.read_csv(CSV_PATH)

qw = df.qw / Q14_SCALE
qx = df.qx / Q14_SCALE
qy = df.qy / Q14_SCALE
qz = df.qz / Q14_SCALE

t_s = ((df.timestamp_ms - df.timestamp_ms.iloc[0]) / 1000.0).values
dt = np.diff(t_s)

# ---------------------------------------------------------------------------
# 2. Odjecie bias + grawitacji - w body frame, przed rotacja (gv[] traktowane
#    jako zweryfikowane/zaufane, zgodnie z ustaleniem)
# ---------------------------------------------------------------------------
lin_ax = (df.ax - ACCEL_BIAS_MG[0] - df.gvx).values
lin_ay = (df.ay - ACCEL_BIAS_MG[1] - df.gvy).values
lin_az = (df.az - ACCEL_BIAS_MG[2] - df.gvz).values

plt.figure()
plt.plot(t_s, lin_az)
plt.title("linAz - body frame, po odjeciu bias+grawitacji, PRZED rotacja")
plt.xlabel("t [s]"); plt.ylabel("mg"); plt.show()

# ---------------------------------------------------------------------------
# 3. Rotacja do world frame - wszystkie trzy osie
# ---------------------------------------------------------------------------
world_ax, world_ay, world_az = rotate_body_to_world(
    qw.values, qx.values, qy.values, qz.values, lin_ax, lin_ay, lin_az)

plt.figure()
plt.plot(t_s, world_az)
plt.title("worldAz - po rotacji do ukladu swiata")
plt.xlabel("t [s]"); plt.ylabel("mg"); plt.show()

# ---------------------------------------------------------------------------
# 4. Wstepne calkowanie samej osi Z (bez ZUPT) - do znalezienia TURNAROUND.
#    Turnaround to z natury pojecie pionowe (najnizszy punkt ruchu), wiec
#    lokalizujemy go wylacznie na podstawie Z, niezaleznie od tego ktora
#    poplioma os pozniej okaze sie "strzalkowa".
# ---------------------------------------------------------------------------
b0, a0 = butter(2, LOWPASS_CUTOFF_HZ, fs=SAMPLE_RATE_HZ, btype="low")
world_az_filt = filtfilt(b0, a0, world_az) * MG_TO_MS2
v_raw_z = integrate_trapezoidal(world_az_filt, dt)

plt.figure()
plt.plot(t_s, v_raw_z)
plt.title("Predkosc Z PRZED ZUPT (widac dryft)")
plt.xlabel("t [s]"); plt.ylabel("m/s"); plt.show()

v_prelim = v_raw_z.copy()
linear_detrend(v_prelim, 0, len(v_prelim) - 1)
pos_prelim = integrate_trapezoidal(v_prelim, dt)

margin = max(1, len(pos_prelim) // 20)
search_slice = slice(margin, len(pos_prelim) - margin)
turnaround_idx = margin + int(np.argmin(pos_prelim[search_slice]))

plt.figure()
plt.plot(t_s, pos_prelim)
plt.axvline(t_s[turnaround_idx], color="r", linestyle="--",
           label=f"TURNAROUND @ {t_s[turnaround_idx]:.3f}s")
plt.legend(); plt.title("Pozycja Z (wstepna) - lokalizacja TURNAROUND")
plt.xlabel("t [s]"); plt.ylabel("m"); plt.show()

# ---------------------------------------------------------------------------
# 5. Wlasciwe przetworzenie WSZYSTKICH trzech osi, z korekcja ZUPT
#    kotwiczona na start/TURNAROUND/koniec
# ---------------------------------------------------------------------------
v_x, pos_x = process_axis(world_ax, dt, turnaround_idx, LOWPASS_CUTOFF_HZ, SAMPLE_RATE_HZ)
v_y, pos_y = process_axis(world_ay, dt, turnaround_idx, LOWPASS_CUTOFF_HZ, SAMPLE_RATE_HZ)
v_z, pos_z = process_axis(world_az, dt, turnaround_idx, LOWPASS_CUTOFF_HZ, SAMPLE_RATE_HZ)

plt.figure()
plt.plot(t_s, v_z, label="Vz")
plt.axvline(t_s[turnaround_idx], color="r", linestyle="--")
plt.legend(); plt.title("Predkosc Z PO korekcji ZUPT (2 odcinki)")
plt.xlabel("t [s]"); plt.ylabel("m/s"); plt.show()

# ---------------------------------------------------------------------------
# 6. Trajektoria 2D - dwa rzuty, kolor = czas, znaczniki start/turnaround/koniec
# ---------------------------------------------------------------------------
def plot_2d_trajectory(ax, pos_h, pos_z, t_s, turnaround_idx, h_label):
    pos_h_cm = pos_h * 100.0
    pos_z_cm = pos_z * 100.0

    sc = ax.scatter(pos_h_cm, pos_z_cm, c=t_s, cmap="viridis", s=12, zorder=2)
    ax.plot(pos_h_cm, pos_z_cm, color="gray", alpha=0.3, linewidth=1, zorder=1)

    ax.scatter(pos_h_cm[0], pos_z_cm[0], color="green", s=100,
              marker="o", label="Start", zorder=3)
    ax.scatter(pos_h_cm[turnaround_idx], pos_z_cm[turnaround_idx],
              color="red", s=100, marker="X", label="Turnaround", zorder=3)
    ax.scatter(pos_h_cm[-1], pos_z_cm[-1], color="blue", s=100,
              marker="s", label="Koniec", zorder=3)

    ax.set_xlabel(f"{h_label} [cm]")
    ax.set_ylabel("Z (pion) [cm]")
    ax.set_title(f"Trajektoria - rzut {h_label}-Z")
    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    ax.grid(alpha=0.3)
    return sc


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
sc1 = plot_2d_trajectory(ax1, pos_x, pos_z, t_s, turnaround_idx, "X")
sc2 = plot_2d_trajectory(ax2, pos_y, pos_z, t_s, turnaround_idx, "Y")
fig.colorbar(sc2, ax=ax2, label="t [s]")
fig.suptitle("Trajektoria 2D urzadzenia - oba rzuty poziome vs pion\n"
            "(ktory pokazuje naturalny luk = plaszczyzna strzalkowa, "
            "ktory zostaje plaski = drift boczny - ocen wizualnie)")
plt.tight_layout()
plt.show()

# ---------------------------------------------------------------------------
# Podsumowanie liczbowe
# ---------------------------------------------------------------------------
print(f"\nTURNAROUND w probce {turnaround_idx} / {len(df)} "
      f"(t={t_s[turnaround_idx]:.3f}s)")
print(f"Peak predkosc koncentryki (Z): {v_z[turnaround_idx:].max():.3f} m/s")
print(f"Zakres pozycji pionowej (ROM): {(pos_z.max() - pos_z.min())*100:.2f} cm")
print(f"Zakres X (amplituda pozioma): {(pos_x.max() - pos_x.min())*100:.2f} cm")
print(f"Zakres Y (amplituda pozioma): {(pos_y.max() - pos_y.min())*100:.2f} cm")
print("Wieksza amplituda pozioma z X/Y to prawdopodobnie plaszczyzna "
      "strzalkowa (naturalny luk ruchu); mniejsza to drift boczny.")