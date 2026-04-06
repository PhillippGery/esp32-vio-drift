#!/usr/bin/env python3
"""
view_flow.py — Capture two frames from DRIFT Node 1 and visualize
FAST corners + Lucas-Kanade optical flow arrows.

Usage:
  python tools/view_flow.py --port COM5

Sends two 'c' commands: first establishes the reference frame,
second captures with flow vectors from the previous frame.

Requires: pip install pyserial Pillow numpy
"""

import argparse
import serial
import struct
import time
import numpy as np
from PIL import Image, ImageDraw

WIDTH  = 320
HEIGHT = 240
FRAME_BYTES = WIDTH * HEIGHT

FRAME_SYNC  = b'\xFF\xD8\xBE\xEF'
CORNER_SYNC = b'\xC0\x52\x4E\x52'
FLOW_SYNC   = b'\xF1\x0E\xDA\x7A'

def wait_for_sync(ser, marker, timeout=15):
    buf = b''
    start = time.time()
    while time.time() - start < timeout:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf[-4:] == marker:
            return True
        if len(buf) > 4096:
            buf = buf[-4:]
    return False

def read_packet(ser):
    # 1. Frame
    if not wait_for_sync(ser, FRAME_SYNC):
        return None, None, None
    raw = ser.read(FRAME_BYTES)
    if len(raw) != FRAME_BYTES:
        return None, None, None

    # 2. Corners
    if not wait_for_sync(ser, CORNER_SYNC):
        return raw, None, None
    n_corners = struct.unpack('<H', ser.read(2))[0]
    corners = []
    for _ in range(n_corners):
        x, y, score = struct.unpack('<HHh', ser.read(6))
        corners.append((x, y, score))

    # 3. Flow
    if not wait_for_sync(ser, FLOW_SYNC):
        return raw, corners, None
    n_flow = struct.unpack('<H', ser.read(2))[0]
    flow = []
    for _ in range(n_flow):
        px, py, dx, dy = struct.unpack('<ffff', ser.read(16))
        flow.append((px, py, dx, dy))

    return raw, corners, flow

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--output', default='drift_flow.png')
    parser.add_argument('--scale', type=float, default=3.0,
                        help='Arrow scale factor for visibility')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=15)
    time.sleep(0.5)

    # Frame 1: establish reference (no flow yet)
    print("Frame 1 (reference)...")
    ser.write(b'c')
    raw1, corners1, flow1 = read_packet(ser)
    if raw1 is None:
        print("ERROR: Failed to capture frame 1")
        ser.close()
        return
    print(f"  {len(corners1)} corners captured")

    # Frame 2: captures flow from frame 1 corners
    time.sleep(0.3)
    print("Frame 2 (with flow)...")
    ser.write(b'c')
    raw2, corners2, flow2 = read_packet(ser)
    if raw2 is None:
        print("ERROR: Failed to capture frame 2")
        ser.close()
        return
    ser.close()

    n_flow = len(flow2) if flow2 else 0
    print(f"  {len(corners2)} corners, {n_flow} flow vectors")

    # Build visualization
    pixels = np.frombuffer(raw2, dtype=np.uint8).reshape((HEIGHT, WIDTH))
    img = Image.fromarray(pixels, mode='L').convert('RGB')
    draw = ImageDraw.Draw(img)

    # Draw current corners (green)
    for x, y, score in corners2:
        r = 3
        draw.ellipse([x - r, y - r, x + r, y + r], outline='lime', width=1)

    # Draw flow arrows
    if flow2:
        for px, py, dx, dy in flow2:
            ex = px + dx * args.scale
            ey = py + dy * args.scale
            # Arrow shaft (yellow)
            draw.line([px, py, ex, ey], fill='yellow', width=2)
            # Arrowhead (red dot)
            draw.ellipse([ex - 2, ey - 2, ex + 2, ey + 2], fill='red')
            # Origin (cyan dot)
            draw.ellipse([px - 1, py - 1, px + 1, py + 1], fill='cyan')

    # Info text
    draw.text((5, 5),
              f"FAST: {len(corners2)} corners | LK: {n_flow} flow vectors",
              fill='lime')

    img.save(args.output)
    print(f"\nSaved to {args.output}")
    img.show()

if __name__ == '__main__':
    main()
