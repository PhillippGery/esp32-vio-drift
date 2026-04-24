#!/usr/bin/env python3
"""
live_view.py — Real-time camera + FAST + LK flow viewer for DRIFT Node 1.

Shows a live window with:
  - Grayscale camera feed
  - Green circles = FAST corners
  - Red arrows = optical flow vectors

Usage:
  1. Close PlatformIO serial monitor
  2. python tools/live_view.py --port COM5

Press 'q' to quit, 's' to save a screenshot.

Requires: pip install pyserial opencv-python numpy
"""

import argparse
import serial
import struct
import numpy as np
import cv2
import time

WIDTH  = 320
HEIGHT = 240
FRAME_BYTES = WIDTH * HEIGHT

FRAME_SYNC  = b'\xFF\xD8\xBE\xEF'
CORNER_SYNC = b'\xC0\x52\x4E\x52'
FLOW_SYNC   = b'\xF1\x0E\xDA\x7A'

def wait_for_sync(ser, marker, timeout=3.0):
    """Read bytes until sync marker found, with timeout."""
    buf = b''
    start = time.time()
    while time.time() - start < timeout:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf[-4:] == marker:
            return True
        if len(buf) > 2048:
            buf = buf[-4:]
    return False

def grab_frame_and_data(ser):
    """Send 'c', receive frame + corners + flow vectors."""
    # Flush any stale data
    ser.reset_input_buffer()
    ser.write(b'c')

    # 1. Frame
    if not wait_for_sync(ser, FRAME_SYNC):
        return None, [], []
    raw = ser.read(FRAME_BYTES)
    if len(raw) != FRAME_BYTES:
        return None, [], []

    # 2. Corners
    corners = []
    if wait_for_sync(ser, CORNER_SYNC, timeout=2.0):
        data = ser.read(2)
        if len(data) == 2:
            n_corners = struct.unpack('<H', data)[0]
            for _ in range(n_corners):
                cdata = ser.read(6)
                if len(cdata) == 6:
                    x, y, score = struct.unpack('<HHh', cdata)
                    corners.append((x, y, score))

    # 3. Flow vectors
    flows = []
    if wait_for_sync(ser, FLOW_SYNC, timeout=2.0):
        data = ser.read(2)
        if len(data) == 2:
            n_flow = struct.unpack('<H', data)[0]
            for _ in range(n_flow):
                fdata = ser.read(16)
                if len(fdata) == 16:
                    px, py, dx, dy = struct.unpack('<ffff', fdata)
                    flows.append((px, py, dx, dy))

    pixels = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT, WIDTH))
    return pixels, corners, flows

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--scale', type=int, default=2,
                        help='Display scale factor (default 2x)')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(1)  # Let the board boot

    print("Live viewer started. Press 'q' to quit, 's' to screenshot.")

    frame_count = 0
    fps_start = time.time()
    fps = 0

    while True:
        pixels, corners, flows = grab_frame_and_data(ser)

        if pixels is None:
            continue

        # Convert to BGR for colored overlays
        frame = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)

        # Draw flow vectors (red arrows)
        for px, py, dx, dy in flows:
            pt1 = (int(px), int(py))
            pt2 = (int(px + dx), int(py + dy))
            cv2.arrowedLine(frame, pt1, pt2, (0, 0, 255), 2, tipLength=0.3)
            cv2.circle(frame, pt1, 2, (0, 255, 255), -1)  # Yellow origin

        # Draw corners (green circles)
        for x, y, score in corners:
            cv2.circle(frame, (x, y), 4, (0, 255, 0), 1)
            cv2.drawMarker(frame, (x, y), (0, 255, 0),
                          cv2.MARKER_CROSS, 6, 1)

        # FPS counter
        frame_count += 1
        elapsed = time.time() - fps_start
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            fps_start = time.time()

        # Info overlay
        cv2.putText(frame, f"DRIFT | {len(corners)} corners | {len(flows)} flow | {fps:.1f} FPS",
                    (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Scale up for easier viewing
        display = cv2.resize(frame, (WIDTH * args.scale, HEIGHT * args.scale),
                            interpolation=cv2.INTER_NEAREST)

        cv2.imshow("PROJECT DRIFT - Live View", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"drift_screenshot_{int(time.time())}.png"
            cv2.imwrite(fname, display)
            print(f"Screenshot saved: {fname}")

    ser.close()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == '__main__':
    main()
