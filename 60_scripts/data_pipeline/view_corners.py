#!/usr/bin/env python3
"""
view_corners.py — Capture a frame + FAST corners from DRIFT Node 1
and display them overlaid on the image.

Usage:
  python tools/view_corners.py --port COM5

Requires: pip install pyserial Pillow numpy
"""

import argparse
import serial
import struct
import numpy as np
from PIL import Image, ImageDraw

WIDTH  = 320
HEIGHT = 240
FRAME_BYTES = WIDTH * HEIGHT

FRAME_SYNC  = b'\xFF\xD8\xBE\xEF'
CORNER_SYNC = b'\xC0\x52\x4E\x52'

def wait_for_sync(ser, marker):
    """Read bytes until the 4-byte sync marker is found."""
    buf = b''
    while True:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf[-4:] == marker:
            return
        if len(buf) > 2048:
            buf = buf[-4:]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--output', default='drift_corners.png')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=15)

    # Trigger capture
    ser.write(b'c')
    print("Waiting for frame...")

    # 1. Read frame
    wait_for_sync(ser, FRAME_SYNC)
    raw = ser.read(FRAME_BYTES)
    if len(raw) != FRAME_BYTES:
        print(f"ERROR: Got {len(raw)} frame bytes, expected {FRAME_BYTES}")
        ser.close()
        return
    print(f"Frame received ({FRAME_BYTES} bytes)")

    # 2. Read corners
    wait_for_sync(ser, CORNER_SYNC)
    count_bytes = ser.read(2)
    n_corners = struct.unpack('<H', count_bytes)[0]
    print(f"Reading {n_corners} corners...")

    corners = []
    for _ in range(n_corners):
        data = ser.read(6)  # x(2) + y(2) + score(2)
        x, y, score = struct.unpack('<HHh', data)
        corners.append((x, y, score))

    ser.close()

    # 3. Build image with corner overlay
    pixels = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH))
    # Convert grayscale to RGB so we can draw colored markers
    img = Image.fromarray(pixels, mode='L').convert('RGB')
    draw = ImageDraw.Draw(img)

    # Draw corners — green circles with crosshairs
    r = 4  # Circle radius
    for x, y, score in corners:
        # Circle
        draw.ellipse([x - r, y - r, x + r, y + r], outline='lime', width=1)
        # Crosshair
        draw.line([x - r, y, x + r, y], fill='lime', width=1)
        draw.line([x, y - r, x, y + r], fill='lime', width=1)

    # Add info text
    draw.text((5, 5), f"FAST-9 | {n_corners} corners | threshold=25", fill='lime')

    img.save(args.output)
    print(f"Saved to {args.output} — {n_corners} corners detected")
    img.show()

if __name__ == '__main__':
    main()
