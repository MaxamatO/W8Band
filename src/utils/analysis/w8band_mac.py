"""
w8band_receiver.py
==================
Odbiera dane z W8Band (nRF52 + LSM6DSV16X) przez BLE i zapisuje do CSV.

Packet format (28 bytes, little-endian):
  float32 Qw, Qx, Qy, Qz   (16 B)
  int16   Ax, Ay, Az        ( 6 B)  [mg]
  uint16  _pad              ( 2 B)
  uint32  timestamp_ms      ( 4 B)
Struct: '<ffffhhhHI'
"""

import asyncio
import struct
import csv
import sys
from bleak import BleakClient, BleakScanner

DEVICE_NAME     = "W8Band"
CHAR_CTRL_UUID  = "19B10001-E8F2-537E-4F6C-D104768A1214"
CHAR_DATA_UUID  = "19B10002-E8F2-537E-4F6C-D104768A1214"

FRAME_FMT  = '<ffffhhhHI'   # Qw Qx Qy Qz Ax Ay Az _pad ts
FRAME_SIZE = struct.calcsize(FRAME_FMT)   # == 28
assert FRAME_SIZE == 28, f"Oczekiwano 28, got {FRAME_SIZE}"

OUTPUT_CSV = "w8band_data.csv"

frames: list[tuple] = []
total_raw_bytes = 0


def on_notify(sender, data: bytearray):
    """
    Handler BLE NOTIFY – wywoływany w wątku asyncio.
    Jeden pakiet może zawierać 1–4 próbki (28–112 bajtów).
    """
    global total_raw_bytes
    total_raw_bytes += len(data)

    if len(data) % FRAME_SIZE != 0:
        print(f"  [WARN] Pakiet {len(data)} B nie jest wielokrotnością {FRAME_SIZE} – ignoruję resztę")

    for offset in range(0, len(data) - FRAME_SIZE + 1, FRAME_SIZE):
        chunk = data[offset : offset + FRAME_SIZE]
        if len(chunk) != FRAME_SIZE:
            break
        unpacked = struct.unpack(FRAME_FMT, chunk)
        # unpacked: (Qw, Qx, Qy, Qz, Ax, Ay, Az, _pad, timestamp_ms)
        frames.append(unpacked)

    # Postęp co 120 próbek (~1 s)
    if len(frames) % 120 == 0 and len(frames) > 0:
        ts = frames[-1][8]
        print(f"  Odebrano {len(frames)} próbek | ostatni ts={ts} ms")


async def main():
    print(f"Skanowanie '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if not device:
        print("Nie znaleziono urządzenia. Sprawdź czy nRF jest włączony i reklamuje się.")
        sys.exit(1)

    print(f"Znaleziono: {device.name}  [{device.address}]")

    async with BleakClient(device, timeout=20.0) as client:
        print("Połączono!")

        # 1. Subskrybuj NOTIFY
        await client.start_notify(CHAR_DATA_UUID, on_notify)
        print("Nasłuch włączony.")

        # 2. Wyślij START (0x01)
        print("Wysyłam START → masz 8 sekund na ruch...")
        await client.write_gatt_char(CHAR_CTRL_UUID, bytearray([0x01]), response=True)

        # 3. Czekaj na nagranie (8 s) + przesyłanie (~4 s zapas)
        RECORDING_S   = 8.0
        TRANSFER_S    = 5.0
        TOTAL_WAIT_S  = RECORDING_S + TRANSFER_S

        for elapsed in range(int(TOTAL_WAIT_S)):
            await asyncio.sleep(1.0)
            remaining = int(RECORDING_S) - elapsed
            if remaining > 0:
                print(f"  Nagrywanie... {remaining} s")
            else:
                print(f"  Przesyłanie danych... ({elapsed - int(RECORDING_S) + 1}/{int(TRANSFER_S)} s)")

        # 4. Zatrzymaj notyfikacje
        await client.stop_notify(CHAR_DATA_UUID)
        print(f"\nOdbiór zakończony. Łącznie {len(frames)} próbek ({total_raw_bytes} B surowych danych).")

    # 5. Zapisz CSV
    if not frames:
        print("Brak danych do zapisu.")
        return

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Qw", "Qx", "Qy", "Qz", "Ax", "Ay", "Az", "timestamp_ms"])
        for row in frames:
            # row: (Qw, Qx, Qy, Qz, Ax, Ay, Az, _pad, timestamp_ms)
            writer.writerow([
                f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}",
                row[4], row[5], row[6],
                row[8]   # pomijamy _pad (row[7])
            ])

    print(f"Zapisano do: {OUTPUT_CSV}")

    # Sprawdzenie ciągłości timestampów
    ts_list = [r[8] for r in frames]
    diffs = [ts_list[i+1] - ts_list[i] for i in range(len(ts_list)-1)]
    if diffs:
        avg_dt = sum(diffs) / len(diffs)
        max_dt = max(diffs)
        print(f"Średni dt między próbkami: {avg_dt:.1f} ms  (oczekiwane ~{1000/120:.1f} ms @ 120 Hz)")
        print(f"Maks. dt: {max_dt} ms", end="")
        if max_dt > 50:
            print("  ← UWAGA: duże przerwy mogą oznaczać utratę pakietów BLE!")
        else:
            print()

if __name__ == "__main__":
    asyncio.run(main())