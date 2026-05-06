#!/usr/bin/env python3
"""
visualize_live.py – Real-time visualizer with Trial Tracking, Spacebar Exit,
                     White Text Fix, and Automated GIF Recording.
"""

import argparse
import csv
import json
import socket
import threading
import time
import os
import sys
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.animation import FuncAnimation
from PIL import Image

# ---------------------------------------------------------------------------
# CLI & Constants
# ---------------------------------------------------------------------------

BG_DARK = "#1E1E1E"
BG_AXES = "#2B2B2B"
FG_MAIN = "#00FFCC"
FG_TEXT = "#FFFFFF"  # Forced pure white for readability
FG_TICK = "#FFFFFF"  # Forced pure white for axis labels
GRID_COL = "#3A3A3A"
BUF = 150


def parse_args():
    parser = argparse.ArgumentParser(description="Live DRIFT visualizer.")
    parser.add_argument("--port", type=int, default=4210)
    parser.add_argument("--node", type=int, default=None)
    parser.add_argument("--csv", type=str, default="drift_trials.csv")
    parser.add_argument("--buf", type=int, default=500)
    return parser.parse_args()


def _get(pkt, key):
    v = pkt.get(key)
    return float(v) if v is not None else 0.0


# ---------------------------------------------------------------------------
# UDP Receiver
# ---------------------------------------------------------------------------

class UdpReceiver(threading.Thread):
    def __init__(self, port, node_filter, packet_queue, csv_path):
        super().__init__(daemon=True)
        self.port = port
        self.node_filter = node_filter
        self.queue = packet_queue
        self.csv_path = csv_path

        self.trial_number = self._get_next_trial_number()

        file_exists = os.path.isfile(self.csv_path)
        self.csv_file = open(self.csv_path, "a", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)

        if not file_exists or os.stat(self.csv_path).st_size == 0:
            self.csv_writer.writerow(["trial_number", "receive_time", "node", "ts", "px", "py", "yaw", "temp"])

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.sock.bind(("0.0.0.0", self.port))

        self.received = 0
        self._stop_event = threading.Event()

    def _get_next_trial_number(self):
        if not os.path.exists(self.csv_path): return 1
        try:
            with open(self.csv_path, "r", encoding="utf-8") as f:
                lines = list(csv.reader(f))
                if len(lines) <= 1: return 1
                return int(lines[-1][0]) + 1
        except:
            return 1

    def stop(self):
        self._stop_event.set()

    def run(self):
        print(f"[UDP] Trial #{self.trial_number} started. Listening on {self.port}...")
        while not self._stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
                packet = json.loads(data.decode("utf-8").strip())

                node = packet.get("node")
                if self.node_filter is not None and node != self.node_filter:
                    continue

                packet["_source_ip"] = addr[0]
                self.queue.append(packet)
                self.received += 1

                if self.csv_writer:
                    self.csv_writer.writerow([
                        self.trial_number, time.time(), node, packet.get("ts"),
                        packet.get("px"), packet.get("py"), packet.get("yaw"),
                        packet.get("temperature_c")
                    ])
            except:
                continue

        self.csv_file.flush()
        self.csv_file.close()
        print(f"\n[SAVE] Data saved to {os.path.abspath(self.csv_path)}")
        print(f"[EXIT] Trial #{self.trial_number} complete. {self.received} packets logged.")


# ---------------------------------------------------------------------------
# Visualizer (Forced White Ticks & GIF Engine)
# ---------------------------------------------------------------------------

class DRIFTVisualizer:
    def __init__(self, packet_queue, trial_num):
        self.queue = packet_queue
        self.trial_num = trial_num
        self.px_data, self.py_data, self.yaw_data, self.temp_data = [], [], [], []
        self.pkt_count = 0
        self.last_node, self.last_src, self.last_temp = "?", "?", None

        # Buffer to keep track of image frames for the GIF
        self.frames = []

        # Global Matplotlib styling overrides
        plt.rcParams.update({
            "text.color": FG_TEXT,
            "axes.labelcolor": FG_TEXT,
            "xtick.color": FG_TICK,
            "ytick.color": FG_TICK,
            "axes.facecolor": BG_AXES,
            "figure.facecolor": BG_DARK,
            "axes.edgecolor": "#555555"
        })
        self._build_figure()

        # Keyboard Intercept
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    def _on_key(self, event):
        if event.key == ' ':
            print("\n[SHUTDOWN] Spacebar pressed. Finalizing trial asset creation...")
            plt.close(self.fig)

    def _build_figure(self):
        self.fig = plt.figure(figsize=(14, 8))
        gs = gridspec.GridSpec(3, 4, figure=self.fig, wspace=0.4, hspace=0.4)

        # Main Trajectory
        self.ax_traj = self.fig.add_subplot(gs[:, 0:2])
        self.ax_traj.set_title(f"Trial #{self.trial_num} | [SPACE] to Save & Exit", color=FG_MAIN)
        self.line_traj, = self.ax_traj.plot([], [], color=FG_MAIN, lw=2)
        self.dot_now, = self.ax_traj.plot([], [], "ro")
        self.ax_traj.grid(True, color=GRID_COL)

        # Subplots
        self.ax_pos = self.fig.add_subplot(gs[0, 2])
        self.ax_pos.set_title("Position X/Y", loc="left", fontsize=9)
        self.line_px, = self.ax_pos.plot([], [], color="#FF5555", label="px")
        self.line_py, = self.ax_pos.plot([], [], color="#55FF55", label="py")
        self.ax_pos.legend(fontsize=7, loc="upper left", facecolor=BG_DARK, edgecolor="#555")

        self.ax_yaw = self.fig.add_subplot(gs[1, 2])
        self.ax_yaw.set_title("Yaw (Heading)", loc="left", fontsize=9)
        self.line_yaw, = self.ax_yaw.plot([], [], color="#FFAA00")

        self.ax_temp = self.fig.add_subplot(gs[2, 2])
        self.ax_temp.set_title("Temp (°C)", loc="left", fontsize=9)
        self.line_temp, = self.ax_temp.plot([], [], color="#FF8C00")

        # Metrics Text Panel
        self.ax_metrics = self.fig.add_subplot(gs[:, 3])
        self.ax_metrics.axis("off")
        self.metrics_text = self.ax_metrics.text(0, 0.95, "", transform=self.ax_metrics.transAxes,
                                                 family="monospace", va="top", color=FG_MAIN)

        # Critical fix: Force white colors on all numbers explicitly across subplots
        for ax in [self.ax_traj, self.ax_pos, self.ax_yaw, self.ax_temp]:
            ax.tick_params(colors=FG_TICK, which='both')
            ax.grid(True, color=GRID_COL)

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

            self.px_data.append(_get(pkt, "px"))
            self.py_data.append(_get(pkt, "py"))
            self.yaw_data.append(_get(pkt, "yaw"))

            temp = pkt.get("temperature_c")
            if temp is not None:
                self.last_temp = float(temp)
                self.temp_data.append(self.last_temp)
            self.last_node, self.last_src = pkt.get("node", "?"), pkt.get("_source_ip", "?")

        if not self.px_data: return

        # Render visual updates
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
            f"POS X:  {self.px_data[-1]:.3f} m\n"
            f"POS Y:  {self.py_data[-1]:.3f} m\n"
            f"TEMP:   {self.last_temp if self.last_temp else 0:.2f} C\n\n"
            f"[SPACE] to end trial"
        )

        # GIF Generation: If new packets arrived, take a snapshot of the frame
        if new_data:
            self.fig.canvas.draw()
            # Capture the canvas buffer dynamically
            rgba = np.asarray(self.fig.canvas.buffer_rgba())
            self.frames.append(Image.fromarray(rgba).convert("RGB"))

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.show()

        # Compile and generate the GIF once the window drops out of main loop execution
        if self.frames:
            os.makedirs("gifs", exist_ok=True)
            gif_path = os.path.join("gifs", f"trial_{self.trial_num}.gif")
            print(f"[GIF] Processing {len(self.frames)} frames into animated GIF...")

            # Save using Pillow compression to avoid inflating storage limits
            self.frames[0].save(
                gif_path,
                save_all=True,
                append_images=self.frames[1:],
                duration=100,  # 100ms per frame matching frame interval
                loop=0
            )
            print(f"[SAVE] Animation exported to {os.path.abspath(gif_path)}")


# ---------------------------------------------------------------------------
# Main Execution Flow
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
        recv.join(timeout=2.0)


if __name__ == "__main__":
    main()