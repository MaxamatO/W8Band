import asyncio
import struct
import csv
import sys
from bleak import BleakClient, BleakScanner

DEVICE_NAME     = "W8Ban"
CHAR_CTRL_UUID  = "19B10001-E8F2-537E-4F6C-D104768A1214"
CHAR_DATA_UUID  = "19B10002-E8F2-537E-4F6C-D104768A1214"

# NOWY FORMAT (20 bajtów):
# 4x int16 (Quat Q1.14) + 3x int16 (Accel mg) + 1x uint16 (Seq) + 1x uint32 (Timestamp)
FRAME_FMT  = '<hhhhhhhHI'   
FRAME_SIZE = struct.calcsize(FRAME_FMT)   # == 20
assert FRAME_SIZE == 20, f"Oczekiwano 20, got {FRAME_SIZE}"

OUTPUT_CSV = "w8band_data.csv"

frames: list[tuple] = []
total_raw_bytes = 0
expected_total = 0

# Zdarzenie (Event), które odblokuje skrypt, gdy przyjdzie pakiet EOF
transfer_complete = asyncio.Event()


def on_notify(sender, data: bytearray):
    """
    Handler BLE NOTIFY. Odbiera 20-bajtowe (lub wielokrotność) paczki.
    """
    global total_raw_bytes, expected_total
    total_raw_bytes += len(data)

    if len(data) % FRAME_SIZE != 0:
        print(f"  [WARN] Pakiet {len(data)} B nie jest wielokrotnością {FRAME_SIZE} – ucięty?")

    for offset in range(0, len(data) - FRAME_SIZE + 1, FRAME_SIZE):
        chunk = data[offset : offset + FRAME_SIZE]
        
        # Wypakowanie surowych intów
        qw_raw, qx_raw, qy_raw, qz_raw, ax, ay, az, seq, ts = struct.unpack(FRAME_FMT, chunk)

        # --- DETEKCJA PAKIETU EOF ---
        if seq == 0xFFFF:
            expected_total = ts # W pakiecie EOF timestamp przechowuje 'totalSamples' z C++
            print(f"\n[EOF] Płytka zakończyła wysyłanie! Zadeklarowana ilość próbek: {expected_total}")
            transfer_complete.set() # Odblokowujemy główną pętlę!
            return

        # Dekodowanie Quaternionu z formatu Q1.14 (odwrócenie tego co zrobił C++)
        qw = qw_raw / 16384.0
        qx = qx_raw / 16384.0
        qy = qy_raw / 16384.0
        qz = qz_raw / 16384.0

        frames.append((qw, qx, qy, qz, ax, ay, az, seq, ts))

    # Postęp co 100 próbek
    if len(frames) % 100 == 0 and len(frames) > 0:
        print(f"  Odebrano {len(frames)} próbek...")


async def main():
    print(f"Skanowanie '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if not device:
        print("Nie znaleziono urządzenia. Sprawdź czy XIAO jest włączone.")
        sys.exit(1)

    print(f"Znaleziono: {device.name}  [{device.address}]")

    async with BleakClient(device, timeout=20.0) as client:
        print("Połączono!")

        # Reset flag
        frames.clear()
        transfer_complete.clear()

        # 1. Subskrybuj NOTIFY
        await client.start_notify(CHAR_DATA_UUID, on_notify)
        
        # 2. Wyślij START (0x01)
        print("Wysyłam START → Masz 8 sekund na ruch...")
        await client.write_gatt_char(CHAR_CTRL_UUID, bytearray([0x01]), response=True)

        print("Czekam na dane (Nagrywanie -> Przesyłanie).")
        print("Skrypt wyłączy się AUTOMATYCZNIE, gdy nRF52 wyśle sygnał końca.")

        # 3. Dynamiczne oczekiwanie (max 40 sekund jako bezpiecznik)
        try:
            # await.wait_for zamraża pętlę do momentu wywołania 'transfer_complete.set()'
            await asyncio.wait_for(transfer_complete.wait(), timeout=40.0)
            print(f"Zakończono nasłuch na polecenie urządzenia!")
        except asyncio.TimeoutError:
            print("\n[TIMEOUT] Minęło 40s a sygnał EOF nie dotarł. Rozłączam awaryjnie!")

        # 4. Zatrzymaj notyfikacje
        await client.stop_notify(CHAR_DATA_UUID)

    print(f"\nOdbiór zakończony. Zebrano {len(frames)} próbek (Oczekiwano: {expected_total}).")
    if expected_total > 0 and len(frames) != expected_total:
        zgubione = expected_total - len(frames)
        print(f"⚠️ UWAGA: Zgubiono {zgubione} klatek po drodze (Pakietów BLE)!")

    # 5. Zapisz CSV
    if not frames:
        print("Brak danych do zapisu.")
        return

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        # Dodałem 'Seq', żebyś w Excelu/Pythonie mógł sprawdzić, gdzie dokładnie ucięło klatkę!
        writer.writerow(["Qw", "Qx", "Qy", "Qz", "Ax", "Ay", "Az", "Seq", "timestamp_ms"])
        for row in frames:
            writer.writerow([
                f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}",
                row[4], row[5], row[6],
                row[7], row[8]
            ])

    print(f"Zapisano do: {OUTPUT_CSV}")

if __name__ == "__main__":
    asyncio.run(main())