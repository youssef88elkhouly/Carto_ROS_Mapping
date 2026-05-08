import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(description='Capture raw serial bytes from LDS/LiDAR.')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=230400)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--output', default='lidar_raw.bin')
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baudrate, timeout=0.2)
    ser.reset_input_buffer()

    end_time = time.time() + args.seconds
    total = 0
    fa_count = 0
    with open(args.output, 'wb') as f:
        while time.time() < end_time:
            data = ser.read(4096)
            if data:
                total += len(data)
                fa_count += data.count(bytes([0xFA]))
                f.write(data)

    ser.close()
    print(f'Wrote {total} bytes to {args.output}')
    print(f'Count of 0xFA bytes: {fa_count}')
    print('If 0xFA count is 0, the sensor may not be LDS-01/XV11 protocol or terminal/baud settings are wrong.')


if __name__ == '__main__':
    main()
