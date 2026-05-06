#!/usr/bin/env python3
"""
visualize_live.py – Real-time visualizer for processed DRIFT packets.
New Packet Format: {"node": 1, "ts": 123, "px": 1.2, "py": 0.5, "yaw": 0.78, "temperature_c": 24.1}
"""

import argparse
import csv
import json
import socket
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------------------
# CLI & Constants
# ---------------------------------------------------------------------------

BG_DARK = "#1E1E1E"
BG_AXES = "#2B2B2B"
FG_MAIN = "#00FFCC"
FG_TEXT = "#E0E0E0"
FG_TICK = "#B0B0B0"
GRID_COL = "#3A3A3A"
BUF = 150


def parse_args():
    parser = argparse.ArgumentParser(description="Live DRIFT visualizer.")
    parser.add_argument("--port", type=int, default=4210)
    parser.add_argument("--node", type=int, default=None)
    parser.add_argument("--csv", type=str, default=None)
    parser.add_argument("--buf", type=int, default=500)
    return parser.parse_args()


def _get(pkt, key):
    v = pkt.get(key)
    return float(v) if v is not None else 0.0


# ---------------------------------------------------------------------------
# UDP Receiver
# ---------------------------------------------------------------------------

class UdpReceiver(threading.Thread):
    def __init__(self, port, node_filter, packet_queue, csv_path=None):
        super().__init__(daemon=True)
        self.port = port
        self.node_filter = node_filter
        self.queue = packet_queue
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.sock.bind(("0.0.0.0", self.port))

        self.csv_file = None
        self.csv_writer = None
        if csv_path:
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["time", "node", "ts", "px", "py", "yaw", "temp"])

        self.received = 0
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        print(f"[UDP] Listening on port {self.port}...")
        while not self._stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
                text = data.decode("utf-8", errors="replace").strip()
                packet = json.loads(text)

                node = packet.get("node")
                if self.node_filter is not None and node != self.node_filter:
                    continue

                packet["_source_ip"] = addr[0]
                self.queue.append(packet)
                self.received += 1

                if self.csv_writer:
                    self.csv_writer.writerow([
                        time.time(), node, packet.get("ts"),
                        packet.get("px"), packet.get("py"),
                        packet.get("yaw"), packet.get("temperature_c")
                    ])
            except (socket.timeout, json.JSONDecodeError):
                continue
        if self.csv_file: self.csv_file.close()


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

class DRIFTVisualizer:
    def __init__(self, packet_queue):
        self.queue = packet_queue

        # Buffers
        self.px_data, self.py_data = [], []
        self.yaw_data = []
        self.temp_data = []

        self.pkt_count = 0
        self.last_node, self.last_src = "?", "?"
        self.last_temp = None

        plt.rcParams.update({
            "text.color": FG_TEXT, "axes.labelcolor": FG_TEXT,
            "xtick.color": FG_TICK, "ytick.color": FG_TICK,
            "axes.facecolor": BG_AXES, "figure.facecolor": BG_DARK,
        })
        self._build_figure()

    def _build_figure(self):
        self.fig = plt.figure(figsize=(14, 8))
        gs = gridspec.GridSpec(3, 4, figure=self.fig, wspace=0.4, hspace=0.4)

        # 1. Trajectory (Left Side)
        self.ax_traj = self.fig.add_subplot(gs[:, 0:2])
        self.ax_traj.set_title("Live Trajectory (m)", color=FG_MAIN)
        self.line_traj, = self.ax_traj.plot([], [], color=FG_MAIN, lw=2)
        self.dot_now, = self.ax_traj.plot([], [], "ro")
        self.ax_traj.grid(True, color=GRID_COL)

        # 2. Position vs Time
        self.ax_pos = self.fig.add_subplot(gs[0, 2])
        self.ax_pos.set_title("Position X/Y", loc="left", fontsize=9)
        self.line_px, = self.ax_pos.plot([], [], color="#FF5555", label="px")
        self.line_py, = self.ax_pos.plot([], [], color="#55FF55", label="py")
        self.ax_pos.legend(fontsize=7, loc="upper left")

        # 3. Yaw (Heading)
        self.ax_yaw = self.fig.add_subplot(gs[1, 2])
        self.ax_yaw.set_title("Yaw (Heading)", loc="left", fontsize=9)
        self.line_yaw, = self.ax_yaw.plot([], [], color="#FFAA00")

        # 4. Temperature
        self.ax_temp = self.fig.add_subplot(gs[2, 2])
        self.ax_temp.set_title("Temperature (°C)", loc="left", fontsize=9)
        self.line_temp, = self.ax_temp.plot([], [], color="#FF8C00")

        # 5. Metrics
        self.ax_metrics = self.fig.add_subplot(gs[:, 3])
        self.ax_metrics.axis("off")
        self.metrics_text = self.ax_metrics.text(0, 0.95, "", transform=self.ax_metrics.transAxes,
                                                 family="monospace", va="top", color=FG_MAIN)

    def _rescale(self, ax_obj, data):
        if not data: return
        mn, mx = min(data), max(data)
        pad = max(abs(mx - mn) * 0.1, 0.01)
        ax_obj.set_ylim(mn - pad, mx + pad)
        ax_obj.set_xlim(0, len(data))

    def update(self, _frame):
        new_data = False
        while self.queue:
            pkt = self.queue.popleft()
            self.pkt_count += 1
            new_data = True

            px, py = _get(pkt, "px"), _get(pkt, "py")
            yaw = _get(pkt, "yaw")
            temp = pkt.get("temperature_c")

            self.px_data.append(px)
            self.py_data.append(py)
            self.yaw_data.append(yaw)
            if temp is not None:
                self.last_temp = float(temp)
                self.temp_data.append(self.last_temp)

            self.last_node = pkt.get("node", "?")
            self.last_src = pkt.get("_source_ip", "?")

        if not self.px_data: return

        # Update Trajectory
        self.line_traj.set_data(self.px_data, self.py_data)
        self.dot_now.set_data([self.px_data[-1]], [self.py_data[-1]])
        self.ax_traj.relim();
        self.ax_traj.autoscale_view()

        # Update Rolling Plots
        x_axis = np.arange(len(self.px_data[-BUF:]))
        self.line_px.set_data(x_axis, self.px_data[-BUF:])
        self.line_py.set_data(x_axis, self.py_data[-BUF:])
        self._rescale(self.ax_pos, self.px_data[-BUF:] + self.py_data[-BUF:])

        self.line_yaw.set_data(x_axis, self.yaw_data[-BUF:])
        self._rescale(self.ax_yaw, self.yaw_data[-BUF:])

        if self.temp_data:
            t_tail = self.temp_data[-BUF:]
            self.line_temp.set_data(np.arange(len(t_tail)), t_tail)
            self._rescale(self.ax_temp, t_tail)

        # Update Text
        temp_str = f"{self.last_temp:.2f} °C" if self.last_temp else "N/A"
        self.metrics_text.set_text(
            f"DRIFT TELEMETRY\n{'=' * 20}\n"
            f"Node:   {self.last_node}\nIP:     {self.last_src}\n"
            f"Packets:{self.pkt_count}\n\n"
            f"POSITION\n{'=' * 20}\n"
            f"X: {self.px_data[-1]:.3f} m\nY: {self.py_data[-1]:.3f} m\n\n"
            f"HEADING\n{'=' * 20}\n"
            f"Yaw: {self.yaw_data[-1]:.3f}\n\n"
            f"ENVIRONMENT\n{'=' * 20}\n"
            f"Temp: {temp_str}"
        )

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.show()


def main():
    args = parse_args()
    q = deque(maxlen=args.buf)
    recv = UdpReceiver(args.port, args.node, q, args.csv)
    recv.start()
    try:
        DRIFTVisualizer(q).run()
    finally:
        recv.stop()


if __name__ == "__main__":
    main()