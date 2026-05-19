import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.integrate import cumulative_trapezoid

# ==========================================
# 1. WCZYTANIE DANYCH
# ==========================================
kolumny = ['Qw', 'Qx', 'Qy', 'Qz', 'Ax', 'Ay', 'Az']
df = pd.read_csv('trening.csv', names=kolumny)
df = df.apply(pd.to_numeric, errors='coerce').dropna()

CZESTOTLIWOSC_HZ = 120.0
dt = 1.0 / CZESTOTLIWOSC_HZ

kwaterniony = df[['Qx', 'Qy', 'Qz', 'Qw']].values
akceleracja_lokalna = df[['Ax', 'Ay', 'Az']].values / 1000.0 * 9.81

normy = np.linalg.norm(kwaterniony, axis=1)
dobre_wiersze = normy > 0.5
kwaterniony = kwaterniony[dobre_wiersze]
akceleracja_lokalna = akceleracja_lokalna[dobre_wiersze]

# ==========================================
# 2. KOMPENSACJA GRAWITACJI (Z opóźnionym startem)
# ==========================================
rotacje_surowe = R.from_quat(kwaterniony)

# Omijamy pierwsze 0.5s (kliknięcie startu na telefonie)
START_CALIB = int(0.5 * CZESTOTLIWOSC_HZ)
END_CALIB = int(1.0 * CZESTOTLIWOSC_HZ)

if END_CALIB >= len(rotacje_surowe):
    START_CALIB = 0
    END_CALIB = min(20, len(rotacje_surowe))

q_start_inv = rotacje_surowe[START_CALIB].inv()
rotacje_wyrownane = q_start_inv * rotacje_surowe 

akceleracja_globalna = rotacje_wyrownane.apply(akceleracja_lokalna)

# Kalibracja grawitacji
wektor_grawitacji = np.mean(akceleracja_globalna[START_CALIB:END_CALIB], axis=0)
czysta_akceleracja = akceleracja_globalna - wektor_grawitacji

# PROSTE WYGŁADZANIE (Zamiast filtrów Butterwortha, które psują ZUPT)
czysta_akceleracja = pd.DataFrame(czysta_akceleracja).rolling(window=10, min_periods=1, center=True).mean().values

# ==========================================
# 3. ZUPT (Wykrywanie bezruchu)
# ==========================================
num_samples = len(czysta_akceleracja)
predkosc = np.zeros_like(czysta_akceleracja)

# LICZYMY CAŁKOWITĄ SIŁĘ, BY ZOBACZYĆ BŁĄD
magnituda_calkowita = np.linalg.norm(czysta_akceleracja, axis=1)

PROG_PRZYSPIESZENIA = 4.5  # <--- Ten próg będziesz regulować na podstawie prawego wykresu!
OKNO_ZUPT = int(0.15 * CZESTOTLIWOSC_HZ) 
is_stationary = np.zeros(num_samples, dtype=bool)

for i in range(num_samples):
    start_idx = max(0, i - OKNO_ZUPT // 2)
    end_idx = min(num_samples, i + OKNO_ZUPT // 2)
    
    # Sprawdzamy, czy czerwona linia jest pod czarną
    if np.all(magnituda_calkowita[start_idx:end_idx] < PROG_PRZYSPIESZENIA):
        is_stationary[i] = True

# ==========================================
# 4. CAŁKOWANIE (Bez powrotu na start)
# ==========================================
for i in range(1, num_samples):
    if is_stationary[i]:
        predkosc[i] = np.zeros(3) # HAMULEC ZUPT
    else:
        predkosc[i] = predkosc[i-1] + (czysta_akceleracja[i] + czysta_akceleracja[i-1]) / 2.0 * dt

pozycja = cumulative_trapezoid(predkosc, dx=dt, initial=0, axis=0)

# ==========================================
# 5. RYSOWANIE
# ==========================================
plt.figure(figsize=(14, 6))

# Subplot 1: Tor Ruchu
plt.subplot(1, 2, 1)
# Bierzemy X i Z, tak jak zrobiliśmy to wcześniej
plt.plot(pozycja[:, 0] * 100, pozycja[:, 2] * 100, marker='.', markersize=2, linestyle='-', color='dodgerblue')
plt.scatter(pozycja[0, 0] * 100, pozycja[0, 2] * 100, color='green', s=100, label='Start')
plt.scatter(pozycja[-1, 0] * 100, pozycja[-1, 2] * 100, color='red', s=100, label='Koniec')
plt.title("Tor Ruchu (X vs Z)")
plt.xlabel("Przesunięcie w osi X (cm)")
plt.ylabel("Wysokość Z (cm)")
plt.grid(True)
plt.axis('equal')
plt.legend()

# Subplot 2: DIAGNOSTYKA ZUPT (Demaskowanie problemu)
plt.subplot(1, 2, 2)
plt.plot(magnituda_calkowita, label='Magnituda (Całkowity Błąd Osi)', color='red')
plt.axhline(PROG_PRZYSPIESZENIA, color='black', linestyle='--', label='Twój Próg ZUPT')
plt.fill_between(range(num_samples), 0, max(magnituda_calkowita), where=is_stationary, color='green', alpha=0.2, label='ZUPT Active (Wykryto Bezruch)')
plt.title("Dlaczego system nie hamował? (Patrz na czerwoną linię)")
plt.xlabel("Numer próbki")
plt.ylabel("m/s^2")
plt.ylim([0, max(3, max(magnituda_calkowita))])
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()