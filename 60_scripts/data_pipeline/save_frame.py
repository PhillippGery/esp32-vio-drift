#!/usr/bin/env python3
"""
save_frame.py — Receive a raw grayscale frame from DRIFT Node 1 over serial.

Usage:
  1. Flash the test_capture firmware (uncomment SEND_RAW_FRAME in main.cpp)
  2. python save_frame.py --port /dev/ttyACM0
  3. Opens the captured 320x240 grayscale image

Requires: pip install pyserial Pillow
"""

import argparse
import serial
import struct
from PIL import Image
import numpy as np

WIDTH  = 320
HEIGHT = 240
FRAME_BYTES = WIDTH * HEIGHT  # 76800 for grayscale

SYNC_MARKER = b'\xFF\xD8\xBE\xEF'  # Custom sync marker — match in firmware

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--output', default='drift_frame.png')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=10)
    ser.write(b'c')

    print("Waiting for sync marker...")
    buf = b''
    while True:
        buf += ser.read(1)
        if buf[-4:] == SYNC_MARKER:
            break
        # Keep only last 4 bytes to avoid memory growth
        if len(buf) > 1024:
            buf = buf[-4:]

    print(f"Sync found. Reading {FRAME_BYTES} bytes...")
    raw = ser.read(FRAME_BYTES)
    if len(raw) != FRAME_BYTES:
        print(f"ERROR: Got {len(raw)} bytes, expected {FRAME_BYTES}")
        return

    # Convert to image
    pixels = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH))
    img = Image.fromarray(pixels, mode='L')
    img.save(args.output)
    print(f"Saved to {args.output}")
    img.show()

    ser.close()

if __name__ == '__main__':
    main()