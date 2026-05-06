"""
test_viewer.py — Companion for vio_camera.cpp manual test mode.

Saves each frame as PNG with corners and flow. After 5 flow frames,
shows direction + pixel displacement + confidence.

Usage:
  1. Close serial monitor
  2. python test_viewer.py --port COM5
  3. Press ENTER for each frame

Requires: pip install pyserial Pillow numpy
"""

import argparse
import serial
import struct
import numpy as np
from PIL import Image, ImageDraw
import time
import os
import math

WIDTH  = 320
HEIGHT = 240
FRAME_BYTES = WIDTH * HEIGHT
ACCUM_WINDOW = 5
IMG_CX, IMG_CY = 160.0, 120.0

TEST_SYNC   = b'\xDE\xAD\xBE\xEF'
FRAME_SYNC  = b'\xFF\xD8\xBE\xEF'
CORNER_SYNC = b'\xC0\x52\x4E\x52'
FLOW_SYNC   = b'\xF1\x0E\xDA\x7A'

def wait_for_sync(ser, marker, timeout=10.0):
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

def receive_test_frame(ser):
    if not wait_for_sync(ser, TEST_SYNC):
        return None
    fnum = struct.unpack('B', ser.read(1))[0]
    lat_dx = struct.unpack('<f', ser.read(4))[0]
    lat_dy = struct.unpack('<f', ser.read(4))[0]
    radial = struct.unpack('<f', ser.read(4))[0]
    if not wait_for_sync(ser, FRAME_SYNC, timeout=5):
        return None
    raw = ser.read(FRAME_BYTES)
    if len(raw) != FRAME_BYTES:
        return None
    corners = []
    if wait_for_sync(ser, CORNER_SYNC, timeout=3):
        n = struct.unpack('<H', ser.read(2))[0]
        for _ in range(n):
            d = ser.read(6)
            if len(d) == 6:
                x, y, score = struct.unpack('<HHh', d)
                corners.append((x, y, score))
    flows = []
    if wait_for_sync(ser, FLOW_SYNC, timeout=3):
        n = struct.unpack('<H', ser.read(2))[0]
        for _ in range(n):
            d = ser.read(16)
            if len(d) == 16:
                px, py, dx, dy = struct.unpack('<ffff', d)
                flows.append((px, py, dx, dy))
    return {
        'frame_num': fnum,
        'lat_dx': lat_dx, 'lat_dy': lat_dy, 'radial': radial,
        'pixels': np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH)),
        'corners': corners, 'flows': flows
    }

def classify_direction(lat_dx, lat_dy, radial):
    lat_mag = math.sqrt(lat_dx**2 + lat_dy**2)
    rad_mag = abs(radial)
    if lat_mag < 1.0 and rad_mag < 1.0:
        return "STATIONARY"
    if rad_mag > lat_mag and rad_mag > 1.0:
        return "FORWARD" if radial > 0 else "BACKWARD"
    angle = math.atan2(lat_dy, lat_dx) * 180 / math.pi
    if -45 < angle <= 45: return "RIGHT"
    if 45 < angle <= 135: return "DOWN"
    if angle > 135 or angle <= -135: return "LEFT"
    return "UP"

def draw_frame(data, output_dir):
    img = Image.fromarray(data['pixels'], mode='L').convert('RGB')
    draw = ImageDraw.Draw(img)
    fnum = data['frame_num']
    for px, py, dx, dy in data['flows']:
        x0, y0 = int(px), int(py)
        x1, y1 = int(px + dx), int(py + dy)
        draw.ellipse([x0-2, y0-2, x0+2, y0+2], fill='yellow')
        draw.line([(x0, y0), (x1, y1)], fill='red', width=2)
        draw.ellipse([x1-2, y1-2, x1+2, y1+2], fill='red')
    for x, y, score in data['corners']:
        r = 4
        draw.ellipse([x-r, y-r, x+r, y+r], outline='lime', width=1)
        draw.line([x-r, y, x+r, y], fill='lime')
        draw.line([x, y-r, x, y+r], fill='lime')
    n_c = len(data['corners'])
    n_f = len(data['flows'])
    if fnum == 0:
        label = f"Frame {fnum} (BASELINE) | {n_c} corners"
    else:
        label = (f"Frame {fnum} | {n_c} corners | {n_f} flow | "
                 f"lat({data['lat_dx']:+.2f},{data['lat_dy']:+.2f}) "
                 f"rad={data['radial']:+.2f}")
    draw.text((5, 5), label, fill='lime')
    if fnum > 0:
        direction = classify_direction(data['lat_dx'], data['lat_dy'], data['radial'])
        draw.text((5, HEIGHT - 20), f"DIRECTION: {direction}", fill='yellow')
    fname = os.path.join(output_dir, f"test_frame_{fnum}.png")
    fname_big = os.path.join(output_dir, f"test_frame_{fnum}_2x.png")
    img.save(fname)
    img.resize((WIDTH * 2, HEIGHT * 2), Image.NEAREST).save(fname_big)
    print(f"  Saved: {fname} ({n_c} corners, {n_f} flow)")

def print_summary(flow_frames):
    total_dx = sum(f['lat_dx'] for f in flow_frames)
    total_dy = sum(f['lat_dy'] for f in flow_frames)
    total_rad = sum(f['radial'] for f in flow_frames)
    lat_mag = math.sqrt(total_dx**2 + total_dy**2)
    lat_angle = math.atan2(total_dy, total_dx) * 180 / math.pi
    direction = classify_direction(total_dx, total_dy, total_rad)

    # Confidence calculation (matches flow_accumulator.h)
    total_vectors = sum(len(f['flows']) for f in flow_frames)
    vec_score = min(total_vectors / 50.0, 1.0)
    # Variance of per-frame laterals
    n = len(flow_frames)
    mean_dx = total_dx / n
    mean_dy = total_dy / n
    mean_rad = total_rad / n
    var_lat = sum((f['lat_dx'] - mean_dx)**2 + (f['lat_dy'] - mean_dy)**2 for f in flow_frames) / n
    var_rad = sum((f['radial'] - mean_rad)**2 for f in flow_frames) / n
    var_total = math.sqrt(var_lat + var_rad)
    var_score = 1.0 / (1.0 + var_total * 2.0)
    confidence = vec_score * var_score

    print()
    print("=" * 60)
    print(f"   MEASUREMENT RESULT — {len(flow_frames)} FRAMES")
    print("=" * 60)
    print("  Per-frame breakdown:")
    for i, f in enumerate(flow_frames):
        d = classify_direction(f['lat_dx'], f['lat_dy'], f['radial'])
        print(f"    F{i+1}: lat({f['lat_dx']:+6.2f},{f['lat_dy']:+6.2f}) "
              f"rad={f['radial']:+6.2f}  {d:>10s}  [{len(f['flows'])} vec]")
    print("-" * 60)
    print(f"  PIXEL TOTALS:")
    print(f"    Lateral:  dx={total_dx:+.2f}  dy={total_dy:+.2f}  ({lat_mag:.2f} px)")
    print(f"    Radial:   {total_rad:+.2f} px")
    print("-" * 60)
    print(f"  DIRECTION:  {direction}")
    print(f"  ANGLE:      {lat_angle:.1f} degrees (0=right, 90=down)")
    print(f"  CONFIDENCE: {confidence:.2f}")
    print(f"    vector_score:      {vec_score:.2f} ({total_vectors} vectors)")
    print(f"    consistency_score: {var_score:.2f} (variance={var_total:.3f})")
    print("-" * 60)
    print(f"  NOTE: Camera provides DIRECTION + CONFIDENCE.")
    print(f"        IMU provides DISTANCE (meters).")
    print(f"        EKF fuses both.")
    print("=" * 60)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--output', default='test_frames')
    args = parser.parse_args()
    os.makedirs(args.output, exist_ok=True)

    print(f"Connecting to {args.port}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(1)

    print("\n=== DRIFT Camera Test Viewer ===")
    print(f"Saving frames to: {args.output}/\n")

    ser.write(b't')
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("Ready! Press ENTER for each frame.\n")
    print(">>> Press ENTER for baseline (don't move camera)")

    flow_frames = []
    test_round = 1

    while True:
        fnum = len(flow_frames) + (1 if flow_frames or test_round > 1 else 0)
        user_input = input(f"\n[Frame {fnum}] ENTER=capture, q=quit: ")
        if user_input.lower() == 'q':
            break

        ser.write(b' ')
        print("  Receiving...")
        data = receive_test_frame(ser)
        if data is None:
            print("  ERROR: No data. Try again.")
            continue

        draw_frame(data, args.output)

        if data['frame_num'] == 0:
            print(f"\n  Baseline captured! Move camera, then press ENTER.")
            flow_frames = []
        else:
            flow_frames.append(data)
            d = classify_direction(data['lat_dx'], data['lat_dy'], data['radial'])
            print(f"  lat({data['lat_dx']:+.2f},{data['lat_dy']:+.2f}) "
                  f"rad={data['radial']:+.2f}  DIR={d}")

            if len(flow_frames) >= ACCUM_WINDOW:
                print_summary(flow_frames)
                flow_frames = []
                test_round += 1
                print("\nENTER = new test, q = quit")

    if flow_frames:
        print(f"\n(Partial: {len(flow_frames)} of {ACCUM_WINDOW} frames)")
        print_summary(flow_frames)

    ser.write(b't')
    ser.close()
    print("\nDone.")

if __name__ == '__main__':
    main()