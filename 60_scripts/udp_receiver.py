#!/usr/bin/env python3
import argparse
import csv
import json
import socket
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="Listen for UDP JSON packets from ESP32 nodes on the local network."
    )
    parser.add_argument(
        "--port",
        type=int,
        default=4210,
        help="UDP port to listen on (default: 4210)",
    )
    parser.add_argument(
        "--node",
        type=int,
        default=None,
        help="Optional node ID to filter for",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Optional CSV file path to save received packet rows",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Socket timeout in seconds for polling (default: 0.5)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="Optional run duration in seconds (0 = run until interrupted)",
    )
    return parser.parse_args()


class UdpJsonListener:
    def __init__(self, port: int, node_filter: int = None, csv_path: str = None, timeout: float = 0.5):
        self.port = port
        self.node_filter = node_filter
        self.csv_path = csv_path
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)
        # Listen on all addresses so the broadcast packets are received.
        self.sock.bind(("0.0.0.0", self.port))
        self.csv_file = None
        self.csv_writer = None

        if self.csv_path:
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "receive_time",
                "source_ip",
                "source_port",
                "node",
                "ts",
                "ax",
                "ay",
                "az",
                "gx",
                "gy",
                "gz",
                "raw_json",
            ])

    def close(self):
        if self.csv_file:
            self.csv_file.close()
        self.sock.close()

    def run(self, duration: float = 0):
        print(f"Listening for UDP packets on port {self.port}...")
        if self.node_filter is not None:
            print(f"Filtering for node {self.node_filter}")
        if self.csv_path:
            print(f"Saving received data to {self.csv_path}")

        start_time = time.time()
        received = 0
        while True:
            if duration > 0 and time.time() - start_time >= duration:
                print("Run duration reached, exiting.")
                break
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                print("Interrupted by user.")
                break

            receive_time = time.time()
            source_ip, source_port = addr
            text = data.decode("utf-8", errors="replace").strip()
            try:
                packet = json.loads(text)
            except json.JSONDecodeError:
                print(f"[WARN] Invalid JSON from {source_ip}:{source_port}: {text}")
                continue

            node = packet.get("node")
            if self.node_filter is not None and node != self.node_filter:
                continue

            ts = packet.get("ts")
            ax = packet.get("ax")
            ay = packet.get("ay")
            az = packet.get("az")
            gx = packet.get("gx")
            gy = packet.get("gy")
            gz = packet.get("gz")
            received += 1

            print(f"[{received}] {source_ip}:{source_port} node={node} ts={ts}")
            print(f"    ax={ax} ay={ay} az={az} gx={gx} gy={gy} gz={gz}")

            if self.csv_writer:
                self.csv_writer.writerow([
                    f"{receive_time:.6f}",
                    source_ip,
                    source_port,
                    node,
                    ts,
                    ax,
                    ay,
                    az,
                    gx,
                    gy,
                    gz,
                    text,
                ])

        self.close()
        print(f"Stopped after receiving {received} packet(s).")


def main():
    args = parse_args()
    listener = UdpJsonListener(
        port=args.port,
        node_filter=args.node,
        csv_path=args.csv,
        timeout=args.timeout,
    )
    listener.run(duration=args.duration)


if __name__ == "__main__":
    main()
