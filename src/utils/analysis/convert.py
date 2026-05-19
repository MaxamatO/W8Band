import struct
import csv

INPUT_FILE = "W8Ban(3).txt"
OUTPUT_FILE = "trening.csv"

raw_bytes = bytearray()

print("Parsowanie nowego formatu logów...")

with open(INPUT_FILE, "r") as f:
    for line in f:
        # Szukamy tylko linijek, które faktycznie zawierają odebrane dane
        if 'Application: "' in line and 'value received' in line:
            # Wyciągamy to, co jest między cudzysłowami
            start_idx = line.find('"') + 1
            end_idx = line.rfind('"')
            
            if start_idx > 0 and end_idx > start_idx:
                hex_string = line[start_idx:end_idx]
                # Usuwamy spacje
                hex_string = hex_string.replace(" ", "")
                # Doklejamy czyste bajty do wielkiego bufora
                raw_bytes.extend(bytes.fromhex(hex_string))

print(f"Sklejono {len(raw_bytes)} bajtów.")

# --- Dekodowanie do CSV ---
FRAME_SIZE = 28
DECODE_FORMAT = '<ffffiii'

frames_decoded = 0

with open(OUTPUT_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Qw', 'Qx', 'Qy', 'Qz', 'Ax', 'Ay', 'Az'])
    
    # Tniemy nasz ciągły bufor równo co 28 bajtów
    for i in range(0, len(raw_bytes) - FRAME_SIZE + 1, FRAME_SIZE):
        chunk = raw_bytes[i:i+FRAME_SIZE]
        unpacked = struct.unpack(DECODE_FORMAT, chunk)
        writer.writerow(unpacked)
        frames_decoded += 1

print(f"Sukces! Wygenerowano {frames_decoded} klatek do pliku {OUTPUT_FILE}.")