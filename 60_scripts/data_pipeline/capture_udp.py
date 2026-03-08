#!/usr/bin/env python3
"""
capture_udp.py — Receive JSON sensor packets from all nodes over UDP.

Usage:
    python capture_udp.py --duration 60 --output ../../30_data/raw_imu/

Each node sends a packet like:
    {"node": 1, "ts": 123456, "ax": 0.01, "ay": -0.02, "az": 9.81,
     "gx": 0.001, "gy": -0.002, "gz": 0.0003}

Author: Sam / PROJECT DRIFT
Course: ECE 56800 — Purdue University, Spring 2026
"""

import argparse
import json
import socket
import time
from datetime import datetime
from pathlib import Path

UDP_IP   = "0.0.0.0"
UDP_PORT = 4210
BUFFER   = 1024

def main(duration: float, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    files: dict[int, object] = {}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)

    print(f"[capture_udp] Listening on UDP {UDP_IP}:{UDP_PORT} for {duration}s ...")
    end_time = time.time() + duration

    try:
        while time.time() < end_time:
            try:
                data, _ = sock.recvfrom(BUFFER)
            except socket.timeout:
                continue

            try:
                pkt = json.loads(data.decode())
            except json.JSONDecodeError:
                continue

            node_id = pkt.get("node", 0)
            if node_id not in files:
                path = output_dir / f"node{node_id}_{timestamp}.csv"
                files[node_id] = open(path, "w")
                files[node_id].write("timestamp_ms,node_id,ax,ay,az,gx,gy,gz\n")
                print(f"[capture_udp] Opened {path}")

            f = files[node_id]
            f.write(
                f"{pkt.get('ts',0)},{node_id},"
                f"{pkt.get('ax',0):.6f},{pkt.get('ay',0):.6f},{pkt.get('az',0):.6f},"
                f"{pkt.get('gx',0):.6f},{pkt.get('gy',0):.6f},{pkt.get('gz',0):.6f}\n"
            )
    finally:
        for f in files.values():
            f.close()
        sock.close()
        print(f"[capture_udp] Done. Saved {len(files)} node log(s) to {output_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output",   type=Path,  default=Path("../../30_data/raw_imu"))
    args = parser.parse_args()
    main(args.duration, args.output)
