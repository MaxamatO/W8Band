import asyncio
import struct
import csv
from bleak import BleakClient, BleakScanner

# Nazwa i identyfikatory z Twojego C++
DEVICE_NAME = "W8Band"
CHAR_WRITE_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
CHAR_NOTIFY_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
FRAME_SIZE = 28
DECODE_FORMAT = '<ffffiii'

dane_treningu = []

def odbierz_dane(sender, data):
    """
    Funkcja odpalana automatycznie, gdy Mac odbierze pakiet z nRF52.
    nRF wysyła dane w blokach (nawet do 224 bajtów). Tniemy je co 28.
    """
    for i in range(0, len(data), FRAME_SIZE):
        chunk = data[i:i+FRAME_SIZE]
        if len(chunk) == FRAME_SIZE:
            unpacked = struct.unpack(DECODE_FORMAT, chunk)
            dane_treningu.append(unpacked)

async def main():
    print(f"Skanowanie w poszukiwaniu '{DEVICE_NAME}'...")
    
    # Na Macu nie używamy adresu MAC, bo macOS ze względów bezpieczeństwa
    # ukrywa prawdziwe adresy i generuje losowe UUID. Szukamy po nazwie!
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    
    if not device:
        print("Nie znaleziono urządzenia. Upewnij się, że XIAO jest włączone.")
        return

    print(f"Znaleziono: {device.name} [{device.address}] - Łączenie...")

    async with BleakClient(device) as client:
        print("Połączono!")
        
        # 1. Subskrybujemy powiadomienia (odpowiednik "Listen" w LightBlue)
        await client.start_notify(CHAR_NOTIFY_UUID, odbierz_dane)
        print("Nasłuch włączony.")
        
        # 2. Wysyłamy komendę START (bajt 0x01)
        print("Wysyłam komendę START. Masz 8 sekund na ruch...")
        komenda = bytearray([0x01])
        await client.write_gatt_char(CHAR_WRITE_UUID, komenda)
        
        # 3. Czekamy 12 sekund
        # (8 sekund nagrywania + ~3-4 sekundy na przesłanie danych z bufora przez BLE)
        await asyncio.sleep(12.0)
        
        # 4. Rozłączamy
        await client.stop_notify(CHAR_NOTIFY_UUID)
        print("Koniec odbioru danych.")

    # 5. Zrzut do CSV
    if len(dane_treningu) > 0:
        plik = 'w8band_data_mac.csv'
        with open(plik, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Qx', 'Qy', 'Qz', 'Qw', 'Ax', 'Ay', 'Az'])
            for wiersz in dane_treningu:
                writer.writerow(wiersz)
        print(f"\nSUKCES! Zapisano {len(dane_treningu)} klatek ruchu do pliku {plik}.")
        print("Możesz teraz odpalić skrypt z wykresem trajektorii!")
    else:
        print("\nNie odebrano żadnych danych. Czy nRF52 wysłał dane?")

if __name__ == "__main__":
    asyncio.run(main())