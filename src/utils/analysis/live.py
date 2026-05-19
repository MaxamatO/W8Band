import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from collections import deque

# ==========================================
# 1. KONFIGURACJA
# ==========================================
PORT_COM = '/dev/ttyACM0'  # Zmień na swój port (np. /dev/cu.usbserial... na Macu)
BAUD_RATE = 115200
CZESTOTLIWOSC_HZ = 120.0
dt = 1.0 / CZESTOTLIWOSC_HZ

# Parametry ZUPT i Kalibracji
PROG_PRZYSPIESZENIA = 0.8
ROZMIAR_BUFORA_ZUPT = 15  # Sprawdzamy ostanie ~120ms
PROBKI_KALIBRACJI = 120   # Zbieramy 1 sekundę na kalibrację

# Stan systemu
pozycja = np.zeros(3)
predkosc = np.zeros(3)
wektor_grawitacji = np.zeros(3)
q_start_inv = None

# Bufory do rysowania wykresu (trzymają np. ostatnie 500 punktów)
historia_x = deque(maxlen=500)
historia_z = deque(maxlen=500)
bufor_akceleracji = deque(maxlen=ROZMIAR_BUFORA_ZUPT)

# ==========================================
# 2. PRZYGOTOWANIE WYKRESU NA ŻYWO
# ==========================================
plt.ion() # Włączamy tryb interaktywny Matplotlib
fig, ax = plt.subplots(figsize=(8, 8))
linia, = ax.plot([], [], 'b-', marker='.', markersize=2)
punkt_aktualny, = ax.plot([], [], 'ro', markersize=8) # Czerwona kropka to sztanga

ax.set_xlim(-50, 50)
ax.set_ylim(-20, 100)
ax.set_title("W8Band: Live Tracker Oczekiwanie na kalibrację...")
ax.set_xlabel("Przesunięcie X (cm)")
ax.set_ylabel("Wysokość Z (cm)")
ax.grid(True)

# ==========================================
# 3. GŁÓWNA PĘTLA CZASU RZECZYWISTEGO
# ==========================================
print(f"Otwieram port {PORT_COM}...")
ser = serial.Serial(PORT_COM, BAUD_RATE)

licznik_probek = 0
dane_kalibracyjne = []

try:
    while True:
        # 1. Czekamy na linijkę tekstu z XIAO (np. "Qx,Qy,Qz,Qw,Ax,Ay,Az")
        linia_serial = ser.readline().decode('utf-8').strip()
        wartosci = linia_serial.split(',')
        
        if len(wartosci) != 7:
            continue # Ignorujemy śmieci
            
        try:
            dane = [float(v) for v in wartosci]
        except ValueError:
            continue
            
        quat = dane[0:4] # Qx, Qy, Qz, Qw
        accel_lokalne = np.array(dane[4:7]) / 1000.0 * 9.81
        
        # Filtrujemy zepsute kwaterniony w locie
        if np.linalg.norm(quat) < 0.5:
            continue
            
        rotacja = R.from_quat(quat)

        # --- FAZA 1: KALIBRACJA (Pierwsza sekunda) ---
        if licznik_probek < PROBKI_KALIBRACJI:
            if licznik_probek == 0:
                q_start_inv = rotacja.inv()
                print("NIE RUSZAJ CZUJNIKA! Kalibracja w toku...")
                
            rot_wyr = q_start_inv * rotacja
            accel_glob = rot_wyr.apply(accel_lokalne)
            dane_kalibracyjne.append(accel_glob)
            
            licznik_probek += 1
            if licznik_probek == PROBKI_KALIBRACJI:
                wektor_grawitacji = np.mean(dane_kalibracyjne, axis=0)
                print(f"Kalibracja zakończona! Grawitacja: {wektor_grawitacji}")
                ax.set_title("W8Band: Live Tracker (AKTYWNY)")
            continue

        # --- FAZA 2: ŚLEDZENIE NA ŻYWO ---
        rot_wyr = q_start_inv * rotacja
        accel_glob = rot_wyr.apply(accel_lokalne)
        czyste_accel = accel_glob - wektor_grawitacji
        
        bufor_akceleracji.append(czyste_accel)
        
        # Czekamy aż bufor się zapełni
        if len(bufor_akceleracji) == ROZMIAR_BUFORA_ZUPT:
            # ZUPT: Oceniamy przeszłość, żeby zdecydować o teraźniejszości
            magnitudy = np.linalg.norm(bufor_akceleracji, axis=1)
            
            if np.all(magnitudy < PROG_PRZYSPIESZENIA):
                predkosc = np.zeros(3) # HAMULEC!
            else:
                predkosc += czyste_accel * dt
                
            pozycja += predkosc * dt
            
            # Zapisujemy do rysowania (zamiana na cm)
            historia_x.append(pozycja[0] * 100)
            historia_z.append(pozycja[2] * 100)
            
            # Aktualizujemy wykres co kilka klatek, żeby nie zabić procesora
            if licznik_probek % 5 == 0: 
                linia.set_data(historia_x, historia_z)
                punkt_aktualny.set_data([pozycja[0] * 100], [pozycja[2] * 100])
                fig.canvas.flush_events() # Renderujemy klatkę!
                
        licznik_probek += 1

except KeyboardInterrupt:
    print("Zamykanie...")
    ser.close()
    plt.ioff()
    plt.show()