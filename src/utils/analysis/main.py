"""Rekonstrukcja względnej trajektorii sztangi z jednego powtórzenia IMU.

Wymagania:
    pip install numpy pandas scipy matplotlib

Założenie: plik zawiera pełne pojedyncze powtórzenie wraz z ok. 0.2 s
bezruchu przed i po ruchu. Wynik jest trajektorią *względną*; nie jest
absolutnym pomiarem pozycji w pomieszczeniu.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation


# --- Konfiguracja -----------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
CSV_PATH = SCRIPT_DIR / "dataset3.csv"
OUTPUT_PNG = SCRIPT_DIR / "barbell_trajectory.png"

# Firmware zapisuje nazwane kolumny qw,qx,qy,qz w formacie Q1.14. SciPy
# oczekuje kolejno x,y,z,w, dlatego kolejność jest jawna w funkcji dekodującej.
QUATERNION_SCALE = 16384.0       # 1.0, gdy do CSV zapisujesz już floaty

# Wszystkie trzy grupy muszą mieć te same jednostki fizyczne. Dla Twojej
# próbki ax~995 i gvz~994, czyli są już w mg. bias* też musi być w mg.
ACC_AND_GRAVITY_ARE_MG = True
LOWPASS_CUTOFF_HZ = 10.0
TURNAROUND_HALF_WINDOW_S = 0.1
LSM_TIMESTAMP_TICK_SECONDS = 21.75e-6
MAX_WORLD_GRAVITY_STD_MG = 20.0


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("Nie można znormalizować wektora zerowego.")
    return v / n


def read_timebase(df: pd.DataFrame):
    """Buduje oś czasu z ticków sensora lub starszego timestampu hosta.

    Dla ticków FIFO zachowuje rzeczywiste odstępy między próbkami. Obsługuje
    również przepełnienie 32-bitowego licznika. Stare pliki z timestamp_ms
    pozostają obsługiwane i są wyrównywane do regularnej siatki.
    """
    if "timestamp_ticks" in df.columns:
        ticks = df.timestamp_ticks.to_numpy(dtype=np.uint32)
        delta_ticks = np.diff(ticks.astype(np.int64))
        delta_ticks[delta_ticks < 0] += 2**32
        elapsed_ticks = np.concatenate(([0], np.cumsum(delta_ticks)))
        t = elapsed_ticks * LSM_TIMESTAMP_TICK_SECONDS
        dts = np.diff(t)
        if len(t) < 20:
            raise ValueError("Potrzeba co najmniej 20 próbek (pełnego powtórzenia).")
        if np.any(dts <= 0):
            raise ValueError("Timestamp FIFO musi rosnąć ściśle.")
        dt = np.median(dts)
        if np.max(dts) > 1.5 * dt:
            raise ValueError("Wykryto brakujące próbki w strumieniu FIFO.")
        return t, t, dt

    t_raw = df.timestamp_ms.to_numpy(dtype=float) * 1e-3
    if len(t_raw) < 20:
        raise ValueError("Potrzeba co najmniej 20 próbek (pełnego powtórzenia).")
    dts = np.diff(t_raw)
    if np.any(dts <= 0):
        raise ValueError("timestamp_ms musi rosnąć ściśle.")
    dt = (t_raw[-1] - t_raw[0]) / (len(t_raw) - 1)
    if np.max(dts) > 1.5 * dt:
        raise ValueError(
            "Wykryto brakujące próbki. Nie interpoluj ich w ciemno; "
            "zapisuj timestamp z FIFO albo uzupełnij brakujące próbki."
        )
    t = t_raw[0] + np.arange(len(t_raw)) * dt
    return t_raw, t, dt


def logged_quaternion_to_xyzw(df: pd.DataFrame) -> np.ndarray:
    """Zwraca znormalizowane [x,y,z,w], format oczekiwany przez SciPy."""
    q = df[["qx", "qy", "qz", "qw"]].to_numpy(float) / QUATERNION_SCALE

    # q i -q oznaczają tę samą orientację. Ujednolicenie znaku jest konieczne
    # przed interpolacją do równych chwil czasu.
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] *= -1
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms < 0.5):
        raise ValueError("Kwaternion ma nieprawidłową normę; sprawdź dekodowanie SFLP.")
    return q / norms[:, None]


def interpolate_columns(t_raw, t, values):
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        return np.interp(t, t_raw, values)
    return np.column_stack([np.interp(t, t_raw, values[:, j]) for j in range(values.shape[1])])


def rotate_body_to_world(rotation: Rotation, vectors_body: np.ndarray) -> np.ndarray:
    """Stosuje potwierdzoną na GV konwencję SFLP: body -> world."""
    return rotation.apply(vectors_body)


def integrate(a: np.ndarray, t: np.ndarray) -> np.ndarray:
    return cumulative_trapezoid(a, t, axis=0, initial=0.0)

 
def integrate_with_zero_velocity_anchors(a, t, anchors):
    """Całkuje a->v i odejmuje najmniejszy liniowy dryft spełniający v=0.

    To ZUPT z kotwicami, a nie sztuczne zerowanie pozycji. Pozycja i prędkość
    są ciągłe; zmieniana jest tylko stała składowa przyspieszenia w segmentach.
    """
    anchors = np.unique(np.asarray(anchors, dtype=int))
    v_raw = integrate(a, t)
    correction = np.interp(t, t[anchors], v_raw[anchors])
    return v_raw - correction


def horizontal_basis(e_up: np.ndarray) -> np.ndarray:
    """Tworzy arbitralne, lecz stabilne osie poziome oraz oś pionową e_up."""
    reference = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(reference, e_up)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    e_x = unit(reference - np.dot(reference, e_up) * e_up)
    e_y = unit(np.cross(e_up, e_x))
    return np.column_stack((e_x, e_y, e_up))


def dominant_horizontal_coordinate(position_xyz: np.ndarray) -> np.ndarray:
    """Wybiera główny poziomy kierunek ruchu (rzut strzałkowy/J-curve)."""
    horizontal = position_xyz[:, :2] - position_xyz[0, :2]
    _, _, vt = np.linalg.svd(horizontal - horizontal.mean(axis=0), full_matrices=False)
    direction = vt[0]
    h = horizontal @ direction
    # Znak jest umowny; przyjmujemy, że koniec powtórzenia ma dodatni kierunek.
    return h if h[-1] >= 0 else -h


def plot_trajectory(t, position, velocity, turnaround_idx):
    h = dominant_horizontal_coordinate(position)
    z = position[:, 2] - position[0, 2]
    half = max(1, int(round(TURNAROUND_HALF_WINDOW_S / np.median(np.diff(t)))))
    i0 = max(0, turnaround_idx - half)
    i1 = min(len(t) - 1, turnaround_idx + half)

    fig = plt.figure(figsize=(13, 5.6), constrained_layout=True)
    ax = fig.add_subplot(1, 2, 1)
    ax3 = fig.add_subplot(1, 2, 2, projection="3d")

    # Fazy: opuszczanie, krótka okolica punktu zwrotnego, wyciskanie.
    phases = [(0, i0, "Opuszczanie", "#1f77b4"),
              (i0, i1, "Punkt zwrotny", "#d62728"),
              (i1, len(t) - 1, "Wyciskanie", "#2ca02c")]
    for begin, end, name, color in phases:
        sl = slice(begin, end + 1)
        ax.plot(h[sl] * 100, z[sl] * 100, color=color, lw=2.5, label=name)
        ax3.plot(position[sl, 0] * 100, position[sl, 1] * 100,
                 position[sl, 2] * 100, color=color, lw=2.5, label=name)

    ax.scatter(h[[0, turnaround_idx, -1]] * 100, z[[0, turnaround_idx, -1]] * 100,
               c=["black", "#d62728", "black"], s=[40, 80, 40], zorder=3)
    ax.annotate("start", (h[0] * 100, z[0] * 100), xytext=(5, 5), textcoords="offset points")
    ax.annotate("klatka", (h[turnaround_idx] * 100, z[turnaround_idx] * 100),
                xytext=(5, -15), textcoords="offset points")
    ax.annotate("koniec", (h[-1] * 100, z[-1] * 100), xytext=(5, 5), textcoords="offset points")
    ax.axhline(0, color="0.7", lw=0.8)
    ax.set(title="Trajektoria w płaszczyźnie głównego ruchu", xlabel="poziomo (kierunek PCA) [cm]",
           ylabel="pion, Z↑ [cm]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend()

    ax3.set(title="Trajektoria 3D w układzie świata IMU", xlabel="X [cm]", ylabel="Y [cm]", zlabel="Z↑ [cm]")
    ax3.legend()
    plt.savefig(OUTPUT_PNG, dpi=180, bbox_inches="tight")
    plt.show()

    print(f"Punkt zwrotny: {t[turnaround_idx] - t[0]:.3f} s")
    print(f"ROM pionowy: {(z.max() - z.min()) * 100:.1f} cm")
    print(f"Maks. prędkość w górę: {velocity[turnaround_idx:, 2].max():.3f} m/s")
    print(f"Wykres zapisano: {OUTPUT_PNG.resolve()}")


def main():
    df = pd.read_csv(CSV_PATH)
    required = {"qw", "qx", "qy", "qz", "ax", "ay", "az",
                "gvx", "gvy", "gvz", "biasx", "biasy", "biasz"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Brak kolumn: {sorted(missing)}")
    if not {"timestamp_ticks", "timestamp_ms"} & set(df.columns):
        raise ValueError("Brak kolumny timestamp_ticks lub timestamp_ms.")
    if not ACC_AND_GRAVITY_ARE_MG:
        raise ValueError("Najpierw przelicz ax/ay/az i gv* do wspólnych jednostek mg.")

    t_raw, t, dt = read_timebase(df)
    fs = 1.0 / dt
    if not 0 < LOWPASS_CUTOFF_HZ < fs / 2:
        raise ValueError("LOWPASS_CUTOFF_HZ musi być w zakresie (0, Nyquist).")

    q_raw = logged_quaternion_to_xyzw(df)
    q = interpolate_columns(t_raw, t, q_raw)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    rotation = Rotation.from_quat(q)

    acc_mg = interpolate_columns(t_raw, t, df[["ax", "ay", "az"]].to_numpy())
    gravity_mg = interpolate_columns(t_raw, t, df[["gvx", "gvy", "gvz"]].to_numpy())
    acc_bias_mg = interpolate_columns(t_raw, t, df[["biasx", "biasy", "biasz"]].to_numpy())

    # bias* jest tu offsetem AKCELEROMETRU wyrażonym w mg. Nie podawaj w tych
    # kolumnach biasu żyroskopu SFLP (ma jednostki mdps i nie odejmuje się go od a).
    linear_body_mg = acc_mg - acc_bias_mg - gravity_mg
    linear_world_mg = rotate_body_to_world(rotation, linear_body_mg)
    gravity_world_mg = rotate_body_to_world(rotation, gravity_mg)

    # Dla właściwej konwencji i synchronizacji kwaternionu ten wektor jest
    # prawie stały w świecie. Nie generuj wiarygodnie wyglądającego wykresu,
    # jeżeli ta podstawowa kontrola danych wejściowych nie przechodzi.
    gravity_world_std = np.std(gravity_world_mg, axis=0)
    print("fs = %.2f Hz; std(R·GV) [mg] = %s" %
          (fs, np.array2string(gravity_world_std, precision=1)))
    if np.max(gravity_world_std) > MAX_WORLD_GRAVITY_STD_MG:
        raise ValueError(
            "Niespójny obrót body→world: sprawdź kolejność kwaternionu, "
            "jego kierunek oraz synchronizację rekordów FIFO."
        )

    sos = butter(2, LOWPASS_CUTOFF_HZ, btype="low", fs=fs, output="sos")
    acceleration_world = sosfiltfilt(sos, linear_world_mg, axis=0) * 9.80665 / 1000.0

    # Oś Z powstaje z mediany world-GV; w spoczynku Twój acc i GV są zgodne,
    # więc wskazuje ona "górę" w konwencji przyspieszeniomierza.
    e_up = unit(np.median(gravity_world_mg, axis=0))
    basis = horizontal_basis(e_up)
    acceleration = acceleration_world @ basis

    # Wstępna pozycja służy wyłącznie do lokalizacji minimum (sztanga na klatce).
    v_pre = integrate_with_zero_velocity_anchors(acceleration[:, 2], t, [0, len(t) - 1])
    z_pre = integrate(v_pre, t)
    margin = max(3, int(0.05 * len(t)))
    turnaround_idx = margin + np.argmin(z_pre[margin:-margin])

    # ZUPT: na pionie v=0 także w punkcie zwrotnym. W poziomie kotwice są
    # tylko na początku i końcu, bo w najniższym punkcie bar może wciąż iść w bok.
    velocity = np.empty_like(acceleration)
    for axis in (0, 1):
        velocity[:, axis] = integrate_with_zero_velocity_anchors(acceleration[:, axis], t, [0, len(t) - 1])
    velocity[:, 2] = integrate_with_zero_velocity_anchors(acceleration[:, 2], t,
                                                           [0, turnaround_idx, len(t) - 1])
    position = integrate(velocity, t)
    plot_trajectory(t, position, velocity, turnaround_idx)


if __name__ == "__main__":
    main()
