#!/usr/bin/env python3
"""
visualize_live.py – Real-time visualizer with Trial Tracking.
Saves data to CSV with an incrementing trial_number column.
"""

import argparse
import csv
import json
import socket
import threading
import time
import os
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
    parser = argparse.ArgumentParser(description="Live DRIFT visualizer with Trial Tracking.")
    parser.add_argument("--port", type=int, default=4210)
    parser.add_argument("--node", type=int, default=None)
    # Defaulting to drift_log.csv so trial tracking has a persistent file to check
    parser.add_argument("--csv", type=str, default="drift_trials.csv", help="CSV file path")
    parser.add_argument("--buf", type=int, default=500)
    return parser.parse_args()


def _get(pkt, key):
    v = pkt.get(key)
    return float(v) if v is not None else 0.0


# ---------------------------------------------------------------------------
# UDP Receiver with Trial Logic
# ---------------------------------------------------------------------------

class UdpReceiver(threading.Thread):
    def __init__(self, port, node_filter, packet_queue, csv_path):
        super().__init__(daemon=True)
        self.port = port
        self.node_filter = node_filter
        self.queue = packet_queue
        self.csv_path = csv_path

        # Trial Management Logic
        self.trial_number = self._get_next_trial_number()
        print(f"[DATA] Starting Trial #{self.trial_number}")

        # Initialize CSV (Append mode)
        file_exists = os.path.isfile(self.csv_path)
        self.csv_file = open(self.csv_path, "a", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)

        # Write header only if the file is new
        if not file_exists or os.stat(self.csv_path).st_size == 0:
            self.csv_writer.writerow(["trial_number", "receive_time", "node", "ts", "px", "py", "yaw", "temp"])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.sock.bind(("0.0.0.0", self.port))

        self.received = 0
        self._stop_event = threading.Event()

    def _get_next_trial_number(self):
        """Reads the CSV to find the last trial number used."""
        if not os.path.exists(self.csv_path):
            return 1

        try:
            with open(self.csv_path, "r", encoding="utf-8") as f:
                reader = list(csv.DictReader(f))
                if not reader:
                    return 1
                # Look at the last row's trial_number
                last_trial = reader[-1].get("trial_number")
                return int(last_trial) + 1 if last_trial else 1
        except Exception as e:
            print(f"[WARN] Could not parse trials from CSV: {e}. Defaulting to 1.")
            return 1

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

                # Log with trial number
                if self.csv_writer:
                    self.csv_writer.writerow([
                        self.trial_number,
                        time.time(),
                        node,
                        packet.get("ts"),
                        packet.get("px"),
                        packet.get("py"),
                        packet.get("yaw"),
                        packet.get("temperature_c")
                    ])
            except (socket.timeout, json.JSONDecodeError):
                continue

        self.csv_file.close()
        print(f"[UDP] Trial #{self.trial_number} logged with {self.received} packets.")


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

class DRIFTVisualizer:
    def __init__(self, packet_queue, trial_num):
        self.queue = packet_queue
        self.trial_num = trial_num

        # Buffers
        self.px_data, self.py_data = [], []
        self.yaw_data, self.temp_data = [], []

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

        # Trajectory
        self.ax_traj = self.fig.add_subplot(gs[:, 0:2])
        self.ax_traj.set_title(f"Trial #{self.trial_num} Trajectory", color=FG_MAIN)
        self.line_traj, = self.ax_traj.plot([], [], color=FG_MAIN, lw=2)
        self.dot_now, = self.ax_traj.plot([], [], "ro")
        self.ax_traj.grid(True, color=GRID_COL)

        # Plots
        self.ax_pos = self.fig.add_subplot(gs[0, 2])
        self.line_px, = self.ax_pos.plot([], [], color="#FF5555", label="px")
        self.line_py, = self.ax_pos.plot([], [], color="#55FF55", label="py")
        self.ax_pos.legend(fontsize=7, loc="upper left")

        self.ax_yaw = self.fig.add_subplot(gs[1, 2])
        self.line_yaw, = self.ax_yaw.plot([], [], color="#FFAA00")
        self.ax_yaw.set_title("Yaw", loc="left", fontsize=9)

        self.ax_temp = self.fig.add_subplot(gs[2, 2])
        self.line_temp, = self.ax_temp.plot([], [], color="#FF8C00")
        self.ax_temp.set_title("Temp (°C)", loc="left", fontsize=9)

        # Metrics
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
        while self.queue:
            pkt = self.queue.popleft()
            self.pkt_count += 1

            self.px_data.append(_get(pkt, "px"))
            self.py_data.append(_get(pkt, "py"))
            self.yaw_data.append(_get(pkt, "yaw"))

            temp = pkt.get("temperature_c")
            if temp is not None:
                self.last_temp = float(temp)
                self.temp_data.append(self.last_temp)

            self.last_node = pkt.get("node", "?")
            self.last_src = pkt.get("_source_ip", "?")

        if not self.px_data: return

        # Drawing logic
        self.line_traj.set_data(self.px_data, self.py_data)
        self.dot_now.set_data([self.px_data[-1]], [self.py_data[-1]])
        self.ax_traj.relim();
        self.ax_traj.autoscale_view()

        x_axis = np.arange(len(self.px_data[-BUF:]))
        self.line_px.set_data(x_axis, self.px_data[-BUF:])
        self.line_py.set_data(x_axis, self.py_data[-BUF:])
        self._rescale(self.ax_pos, self.px_data[-BUF:] + self.py_data[-BUF:])

        self.line_yaw.set_data(x_axis, self.yaw_data[-BUF:])
        self._rescale(self.ax_yaw, self.yaw_data[-BUF:])

        if self.temp_data:
            self.line_temp.set_data(np.arange(len(self.temp_data[-BUF:])), self.temp_data[-BUF:])
            self._rescale(self.ax_temp, self.temp_data[-BUF:])

        self.metrics_text.set_text(
            f"TRIAL #{self.trial_num}\n{'=' * 20}\n"
            f"Node:   {self.last_node}\n"
            f"Pkts:   {self.pkt_count}\n\n"
            f"POS X:  {self.px_data[-1]:.3f}\n"
            f"POS Y:  {self.py_data[-1]:.3f}\n"
            f"TEMP:   {self.last_temp if self.last_temp else 0:.2f} C"
        )

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    q = deque(maxlen=args.buf)

    recv = UdpReceiver(args.port, args.node, q, args.csv)
    recv.start()

    try:
        viz = DRIFTVisualizer(q, recv.trial_number)
        viz.run()
    finally:
        recv.stop()


if __name__ == "__main__":
    main()