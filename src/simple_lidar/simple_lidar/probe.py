import argparse
import time
from collections import Counter

import serial


BAUDS = [
    9600,
    19200,
    38400,
    57600,
    115200,
    128000,
    153600,
    230400,
    256000,
    460800,
    500000,
    512000,
    921600,
]


def count_pair(data: bytes, a: int, b: int) -> int:
    return sum(1 for i in range(len(data) - 1) if data[i] == a and data[i + 1] == b)


def count_xv11_packets(data: bytes) -> int:
    # XV11/LDS-01 packets often look like:
    # FA A0..F9 ... 42 bytes
    count = 0
    for i in range(len(data) - 42):
        if data[i] == 0xFA and 0xA0 <= data[i + 1] <= 0xF9:
            count += 1
    return count


def count_ld06_packets(data: bytes) -> int:
    # LD06/LD19 packets usually start with 54 2C and are 47 bytes
    return count_pair(data, 0x54, 0x2C)


def count_ydlidar_packets(data: bytes) -> int:
    # Many YDLIDAR packets have AA 55 header
    return count_pair(data, 0xAA, 0x55)


def count_rplidar_response(data: bytes) -> int:
    # RPLIDAR response descriptor often starts A5 5A
    return count_pair(data, 0xA5, 0x5A)


def printable_ratio(data: bytes) -> float:
    if not data:
        return 0.0
    printable = sum(1 for b in data if 32 <= b <= 126 or b in (9, 10, 13))
    return printable / len(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()

    print(f"Probing {args.port}")
    print("=" * 70)

    for baud in BAUDS:
        try:
            ser = serial.Serial(
                args.port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )

            ser.reset_input_buffer()
            time.sleep(0.2)

            start = time.time()
            chunks = []

            while time.time() - start < args.seconds:
                chunk = ser.read(4096)
                if chunk:
                    chunks.append(chunk)

            ser.close()

            data = b"".join(chunks)
            c = Counter(data)
            top = " ".join(f"{byte:02X}:{count}" for byte, count in c.most_common(8))
            first = " ".join(f"{b:02X}" for b in data[:32])

            print(f"\nBAUD {baud}")
            print(f"  bytes: {len(data)}")
            print(f"  first 32: {first}")
            print(f"  top bytes: {top}")
            print(f"  printable ratio: {printable_ratio(data):.2f}")
            print(f"  XV11/LDS01 FA A0-F9 packets: {count_xv11_packets(data)}")
            print(f"  LD06/LD19 54 2C packets: {count_ld06_packets(data)}")
            print(f"  YDLIDAR AA 55 packets: {count_ydlidar_packets(data)}")
            print(f"  RPLIDAR A5 5A responses: {count_rplidar_response(data)}")

        except Exception as e:
            print(f"\nBAUD {baud}")
            print(f"  ERROR: {e}")


if __name__ == "__main__":
    main()
