#!/usr/bin/env python3
"""
host_read.py  -  Host-side interface for TCD1304 / STM32F411 driver
=======================================================================
Usage:
    python host_read.py --port /dev/ttyACM0 [--tint 10000] [--plot]

Protocol (115200 baud, 8N1):
  'S'             -> single acquisition -> header 0xAA 0x55 LO HI + 3648*uint16
  'I' + 4 bytes   -> set integration time in us (uint32 little-endian)
  'R'             -> read current integration time back (4 bytes)

Dependencies:
    pip install pyserial numpy matplotlib
"""

import argparse
import struct
import sys
import time

import numpy as np
import serial


HEADER_MAGIC = bytes([0xAA, 0x55])
BAUD         = 115200
PIXEL_COUNT  = 3648


def open_port(port: str, timeout: float = 2.0) -> serial.Serial:
    s = serial.Serial(port, BAUD, timeout=timeout)
    time.sleep(0.1)
    s.read_all()   # drain any startup byte ('K')
    return s


def set_integration_time(ser: serial.Serial, us: int) -> None:
    payload = b'I' + struct.pack('<I', int(us))
    ser.write(payload)
    ack = ser.read(1)
    if ack != b'K':
        raise RuntimeError(f"Expected 'K' ACK, got {ack!r}")
    print(f"  Integration time set to {us} us")


def get_integration_time(ser: serial.Serial) -> int:
    ser.write(b'R')
    data = ser.read(4)
    if len(data) < 4:
        raise RuntimeError("Timeout reading integration time")
    return struct.unpack('<I', data)[0]


def acquire(ser: serial.Serial) -> np.ndarray:
    ser.write(b'S')
    hdr = ser.read(2)
    if hdr != HEADER_MAGIC:
        raise RuntimeError(f"Bad header: {hdr!r}")
    count = struct.unpack('<H', ser.read(2))[0]
    if count != PIXEL_COUNT:
        raise RuntimeError(f"Unexpected pixel count {count}")
    raw = ser.read(PIXEL_COUNT * 2)
    if len(raw) != PIXEL_COUNT * 2:
        raise RuntimeError(f"Short read: expected {PIXEL_COUNT*2}, got {len(raw)}")
    return np.frombuffer(raw, dtype='<u2').astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description="TCD1304 / STM32F411 host reader")
    parser.add_argument('--port',  default='/dev/ttyACM0')
    parser.add_argument('--tint',  type=int, default=None,  help='Integration time in us')
    parser.add_argument('--plot',  action='store_true')
    parser.add_argument('--save',  type=str, default=None,  help='Save CSV')
    parser.add_argument('--n',     type=int, default=1,     help='Frames to average')
    args = parser.parse_args()

    print(f"Opening {args.port} @ {BAUD} baud ...")
    try:
        ser = open_port(args.port)
    except serial.SerialException as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    if args.tint is not None:
        set_integration_time(ser, args.tint)

    t_us = get_integration_time(ser)
    print(f"  Current integration time: {t_us} us  ({t_us/1000:.1f} ms)")

    frames = []
    for i in range(args.n):
        print(f"  Acquiring frame {i+1}/{args.n} ...", end='\r')
        frames.append(acquire(ser))
    print()

    spectrum = np.mean(frames, axis=0)
    print(f"  Min: {spectrum.min():.0f}  Max: {spectrum.max():.0f}  "
          f"Mean: {spectrum.mean():.1f}  Std: {spectrum.std():.1f}")

    if args.save:
        np.savetxt(args.save,
                   np.column_stack([np.arange(PIXEL_COUNT), spectrum]),
                   delimiter=',', header='pixel,counts', comments='')
        print(f"  Saved to {args.save}")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(12, 4))
            ax.plot(spectrum, lw=0.8, color='steelblue')
            ax.set_xlabel('Pixel index')
            ax.set_ylabel('ADC counts (12-bit)')
            ax.set_title(f'TCD1304  t_int={t_us} us'
                         + (f'  (avg x{args.n})' if args.n > 1 else ''))
            ax.set_xlim(0, PIXEL_COUNT - 1)
            ax.set_ylim(0, 4096)
            ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("matplotlib not installed; skipping plot")

    ser.close()


if __name__ == '__main__':
    main()
