#!/usr/bin/env python3
"""
visualize_live.py  –  Real-time DRIFT visualizer fed by UDP/JSON packets
from ESP32 IMU nodes.

Packet format expected (JSON over UDP):
  {"node": 1, "ts": 123456, "ax": 0.1, "ay": -0.2, "az": 9.8,
                             "gx": 0.01, "gy": -0.01, "gz": 0.002,
                             "temp": 23.5}

Usage:
  python visualize_live.py [--port 4210] [--node 1] [--csv out.csv]
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
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Live DRIFT visualizer – streams UDP JSON from ESP32 nodes."
    )
    parser.add_argument("--port",    type=int,   default=4210,  help="UDP port (default: 4210)")
    parser.add_argument("--node",    type=int,   default=None,  help="Filter by node ID")
    parser.add_argument("--csv",     type=str,   default=None,  help="Optional CSV log path")
    parser.add_argument("--timeout", type=float, default=0.5,   help="Socket poll timeout (s)")
    parser.add_argument("--buf",     type=int,   default=500,   help="Max queued packets (default: 500)")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# UDP receiver thread
# ---------------------------------------------------------------------------

class UdpReceiver(threading.Thread):
    """
    Background daemon thread.  Receives UDP JSON packets and pushes parsed
    dicts onto `packet_queue` (a thread-safe collections.deque).
    """

    def __init__(self, port: int, node_filter, packet_queue: deque,
                 csv_path: str = None, timeout: float = 0.5):
        super().__init__(daemon=True)
        self.port         = port
        self.node_filter  = node_filter
        self.queue        = packet_queue
        self.csv_path     = csv_path
        self.timeout      = timeout
        self._stop_event  = threading.Event()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)
        self.sock.bind(("0.0.0.0", self.port))

        self._csv_file   = None
        self._csv_writer = None
        if csv_path:
            self._csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "receive_time", "source_ip", "source_port",
                "node", "ts", "ax", "ay", "az", "gx", "gy", "gz", "temp", "raw_json",
            ])

        self.received = 0

    def stop(self):
        self._stop_event.set()

    def run(self):
        print(f"[UDP] Listening on port {self.port}" +
              (f" (node={self.node_filter})" if self.node_filter is not None else ""))

        while not self._stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue

            receive_time = time.time()
            source_ip, source_port = addr
            text = data.decode("utf-8", errors="replace").strip()

            try:
                packet = json.loads(text)
            except json.JSONDecodeError:
                print(f"[WARN] Bad JSON from {source_ip}: {text[:80]}")
                continue

            node = packet.get("node")
            if self.node_filter is not None and node != self.node_filter:
                continue

            packet["_receive_time"] = receive_time
            packet["_source_ip"]    = source_ip
            packet["_source_port"]  = source_port
            self.queue.append(packet)
            self.received += 1

            if self._csv_writer:
                self._csv_writer.writerow([
                    f"{receive_time:.6f}", source_ip, source_port,
                    node, packet.get("ts"),
                    packet.get("ax"), packet.get("ay"), packet.get("az"),
                    packet.get("gx"), packet.get("gy"), packet.get("gz"),
                    packet.get("temp"),
                    text,
                ])

        if self._csv_file:
            self._csv_file.close()
        self.sock.close()
        print(f"[UDP] Stopped. Total received: {self.received}")


# ---------------------------------------------------------------------------
# Minimal dead-reckoning integrator
# ---------------------------------------------------------------------------

class Integrator:
    ZUPT_THRESHOLD = 0.05
    GRAVITY        = 9.81

    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self._last_ts = None

    def update(self, ax, ay, az, ts_ms: float):
        ts_s = ts_ms / 1000.0
        if self._last_ts is None:
            self._last_ts = ts_s
            return self.pos.copy(), self.vel.copy()

        dt = ts_s - self._last_ts
        if dt <= 0 or dt > 1.0:
            self._last_ts = ts_s
            return self.pos.copy(), self.vel.copy()

        self._last_ts = ts_s
        accel = np.array([ax, ay, az - self.GRAVITY])
        self.vel += accel * dt
        if np.linalg.norm(self.vel) < self.ZUPT_THRESHOLD:
            self.vel[:] = 0.0
        self.pos += self.vel * dt
        return self.pos.copy(), self.vel.copy()


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

BG_DARK  = "#1E1E1E"
BG_AXES  = "#2B2B2B"
FG_MAIN  = "#00FFCC"
FG_TEXT  = "#E0E0E0"
FG_TICK  = "#B0B0B0"
GRID_COL = "#3A3A3A"

plt.rcParams.update({
    "font.family":     "DejaVu Sans",   # safe cross-platform fallback
    "text.color":      FG_TEXT,
    "axes.labelcolor": FG_TEXT,
    "xtick.color":     FG_TICK,
    "ytick.color":     FG_TICK,
    "axes.facecolor":  BG_AXES,
    "figure.facecolor": BG_DARK,
    "axes.edgecolor":  "#555555",
})

BUF = 150   # samples shown in rolling plots


def _get(pkt, key):
    """Return float value from packet; 0.0 if key absent or None."""
    v = pkt.get(key)
    return float(v) if v is not None else 0.0


class DRIFTVisualizer:

    def __init__(self, packet_queue: deque):
        self.queue      = packet_queue
        self.integrator = Integrator()

        # Rolling data stored as plain Python lists (append + slice is fast enough)
        self.ax_data  = []   # accel x
        self.ay_data  = []   # accel y
        self.gx_data  = []   # gyro x
        self.gy_data  = []   # gyro y
        self.vx_data  = []   # velocity x
        self.vy_data  = []   # velocity y
        self.temp_data = []  # temperature °C

        self.traj_x  = []
        self.traj_y  = []

        self.total_dist = 0.0
        self.pkt_count  = 0
        self.last_temp  = None
        self.last_node  = "?"
        self.last_src   = "?"

        self._build_figure()

    # ── Figure construction ────────────────────────────────────────────────

    def _build_figure(self):
        self.fig = plt.figure(figsize=(16, 9), facecolor=BG_DARK)
        gs = gridspec.GridSpec(4, 4, figure=self.fig,
                               wspace=0.35, hspace=0.55,
                               left=0.05, right=0.97,
                               top=0.95, bottom=0.06)

        # ── Trajectory (left 2 cols, all rows) ────────────────────────────
        self.ax_traj = self.fig.add_subplot(gs[:, 0:2])
        self.ax_traj.set_facecolor(BG_AXES)
        self.ax_traj.set_title("2D Trajectory (X-Y)", color=FG_MAIN, fontsize=13)
        self.ax_traj.set_xlabel("X (m)")
        self.ax_traj.set_ylabel("Y (m)")
        self.ax_traj.grid(True, color=GRID_COL)
        self.line_traj,  = self.ax_traj.plot([], [], color=FG_MAIN,  lw=2,   label="path")
        self.dot_start,  = self.ax_traj.plot([], [], "o", color="#FFAA00", ms=7, label="start")
        self.ax_traj.legend(facecolor=BG_DARK, edgecolor="#555", loc="lower right", fontsize=8)

        # ── Accel (row 0, col 2) ───────────────────────────────────────────
        self.ax_accel = self.fig.add_subplot(gs[0, 2])
        self.ax_accel.set_facecolor(BG_AXES)
        self.ax_accel.set_title("Accel (m/s²)", loc="left", fontsize=9, color=FG_TEXT)
        self.ax_accel.grid(True, color=GRID_COL, alpha=0.6)
        self.line_ax, = self.ax_accel.plot([], [], color="#FF5555", lw=1.2, label="ax")
        self.line_ay, = self.ax_accel.plot([], [], color="#55FF55", lw=1.2, label="ay")
        self.ax_accel.legend(facecolor=BG_DARK, edgecolor="#555", fontsize=7, loc="upper left")

        # ── Gyro (row 1, col 2) ────────────────────────────────────────────
        self.ax_gyro = self.fig.add_subplot(gs[1, 2])
        self.ax_gyro.set_facecolor(BG_AXES)
        self.ax_gyro.set_title("Gyro (rad/s)", loc="left", fontsize=9, color=FG_TEXT)
        self.ax_gyro.grid(True, color=GRID_COL, alpha=0.6)
        self.line_gx, = self.ax_gyro.plot([], [], color="#FFAA00", lw=1.2, label="gx")
        self.line_gy, = self.ax_gyro.plot([], [], color="#AAFF00", lw=1.2, label="gy")
        self.ax_gyro.legend(facecolor=BG_DARK, edgecolor="#555", fontsize=7, loc="upper left")

        # ── Velocity (row 2, col 2) ────────────────────────────────────────
        self.ax_vel = self.fig.add_subplot(gs[2, 2])
        self.ax_vel.set_facecolor(BG_AXES)
        self.ax_vel.set_title("Velocity (m/s)", loc="left", fontsize=9, color=FG_TEXT)
        self.ax_vel.grid(True, color=GRID_COL, alpha=0.6)
        self.line_vx, = self.ax_vel.plot([], [], color="#FF00FF", lw=1.2, label="vx")
        self.line_vy, = self.ax_vel.plot([], [], color="#00FFFF", lw=1.2, label="vy")
        self.ax_vel.legend(facecolor=BG_DARK, edgecolor="#555", fontsize=7, loc="upper left")

        # ── Temperature (row 3, col 2) ─────────────────────────────────────
        self.ax_temp = self.fig.add_subplot(gs[3, 2])
        self.ax_temp.set_facecolor(BG_AXES)
        self.ax_temp.set_title("Temperature (°C)", loc="left", fontsize=9, color=FG_TEXT)
        self.ax_temp.set_ylabel("°C", fontsize=8)
        self.ax_temp.grid(True, color=GRID_COL, alpha=0.6)
        self.line_temp, = self.ax_temp.plot([], [], color="#FF8C00", lw=1.5, label="temp")
        self.ax_temp.legend(facecolor=BG_DARK, edgecolor="#555", fontsize=7, loc="upper left")

        # ── Metrics text panel (col 3, all rows) ──────────────────────────
        self.ax_metrics = self.fig.add_subplot(gs[:, 3])
        self.ax_metrics.set_facecolor(BG_DARK)
        self.ax_metrics.axis("off")
        self.metrics_text = self.ax_metrics.text(
            0.05, 0.97, "",
            transform=self.ax_metrics.transAxes,
            va="top", fontsize=9.5, color=FG_MAIN,
            family="monospace",
        )

        self.fig.canvas.manager.set_window_title("DRIFT – Waiting for packets…")

    # ── Drain queue ────────────────────────────────────────────────────────

    def _drain_queue(self):
        """Pull all waiting packets, update data lists. Returns True if any processed."""
        got_data = False
        while self.queue:
            pkt = self.queue.popleft()
            self.pkt_count += 1
            got_data = True

            ax_v = _get(pkt, "ax")
            ay_v = _get(pkt, "ay")
            az_v = _get(pkt, "az")
            gx_v = _get(pkt, "gx")
            gy_v = _get(pkt, "gy")
            ts_v = _get(pkt, "ts")

            raw_temp = pkt.get("temp")
            if raw_temp is not None:
                self.last_temp = float(raw_temp)
                self.temp_data.append(self.last_temp)

            pos, vel = self.integrator.update(ax_v, ay_v, az_v, ts_v)

            if self.traj_x:
                dx = pos[0] - self.traj_x[-1]
                dy = pos[1] - self.traj_y[-1]
                self.total_dist += np.sqrt(dx * dx + dy * dy)
            self.traj_x.append(pos[0])
            self.traj_y.append(pos[1])

            self.ax_data.append(ax_v)
            self.ay_data.append(ay_v)
            self.gx_data.append(gx_v)
            self.gy_data.append(gy_v)
            self.vx_data.append(float(vel[0]))
            self.vy_data.append(float(vel[1]))

            self.last_node = pkt.get("node", "?")
            self.last_src  = pkt.get("_source_ip", "?")

        return got_data

    # ── Helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _tail(lst):
        """Return the last BUF elements as a numpy array."""
        return np.array(lst[-BUF:])

    @staticmethod
    def _rescale(ax_obj, ydata):
        """Rescale y-axis to fit ydata with a small margin."""
        if len(ydata) == 0:
            return
        mn, mx = float(np.min(ydata)), float(np.max(ydata))
        pad = max(abs(mx - mn) * 0.1, 1e-6)
        ax_obj.set_ylim(mn - pad, mx + pad)
        ax_obj.set_xlim(0, len(ydata))

    # ── Animation frame ────────────────────────────────────────────────────

    def update(self, _frame):
        if not self._drain_queue():
            self.fig.canvas.manager.set_window_title(
                f"DRIFT – waiting… ({self.pkt_count} received)")
            return

        node = self.last_node
        src  = self.last_src

        self.fig.canvas.manager.set_window_title(
            f"DRIFT – node={node}  src={src}  pkts={self.pkt_count}")

        # ── Trajectory ────────────────────────────────────────────────────
        xs, ys = np.array(self.traj_x), np.array(self.traj_y)
        self.line_traj.set_data(xs, ys)
        if len(xs) > 0:
            self.dot_start.set_data([xs[0]], [ys[0]])
        if len(xs) > 1:
            xpad = max((xs.max() - xs.min()) * 0.1, 0.1)
            ypad = max((ys.max() - ys.min()) * 0.1, 0.1)
            self.ax_traj.set_xlim(xs.min() - xpad, xs.max() + xpad)
            self.ax_traj.set_ylim(ys.min() - ypad, ys.max() + ypad)

        # ── Rolling sensor plots ───────────────────────────────────────────
        pairs = [
            (self.ax_accel, self.line_ax, self.ax_data,
                            self.line_ay, self.ay_data),
            (self.ax_gyro,  self.line_gx, self.gx_data,
                            self.line_gy, self.gy_data),
            (self.ax_vel,   self.line_vx, self.vx_data,
                            self.line_vy, self.vy_data),
        ]
        for subplot, line0, data0, line1, data1 in pairs:
            t0 = self._tail(data0)
            t1 = self._tail(data1)
            xs_idx = np.arange(len(t0))
            line0.set_data(xs_idx, t0)
            line1.set_data(xs_idx, t1)
            combined = np.concatenate([t0, t1])
            self._rescale(subplot, combined)

        # ── Temperature plot ───────────────────────────────────────────────
        if self.temp_data:
            t_arr = self._tail(self.temp_data)
            self.line_temp.set_data(np.arange(len(t_arr)), t_arr)
            self._rescale(self.ax_temp, t_arr)

        # ── Metrics panel ─────────────────────────────────────────────────
        vel  = np.array([self.vx_data[-1], self.vy_data[-1]]) if self.vx_data else np.zeros(2)
        acc  = np.array([self.ax_data[-1], self.ay_data[-1]]) if self.ax_data else np.zeros(2)
        gyr  = np.array([self.gx_data[-1], self.gy_data[-1]]) if self.gx_data else np.zeros(2)
        pos  = np.array([self.traj_x[-1],  self.traj_y[-1]])  if self.traj_x  else np.zeros(2)
        spd  = float(np.linalg.norm(vel))
        temp_str = f"{self.last_temp:.2f} °C" if self.last_temp is not None else "N/A"

        self.metrics_text.set_text(
            f"LIVE TELEMETRY\n"
            f"{'='*24}\n"
            f"Node:       {node}\n"
            f"Source:     {src}\n"
            f"Packets:    {self.pkt_count}\n"
            f"Dist (XY):  {self.total_dist:.3f} m\n"
            f"\n"
            f"POSITION\n"
            f"{'='*24}\n"
            f"px: {pos[0]:>8.3f} m\n"
            f"py: {pos[1]:>8.3f} m\n"
            f"\n"
            f"VELOCITY\n"
            f"{'='*24}\n"
            f"vx: {vel[0]:>8.3f} m/s\n"
            f"vy: {vel[1]:>8.3f} m/s\n"
            f"|v|:{spd:>8.3f} m/s\n"
            f"\n"
            f"ACCELEROMETER\n"
            f"{'='*24}\n"
            f"ax: {acc[0]:>8.3f}\n"
            f"ay: {acc[1]:>8.3f}\n"
            f"\n"
            f"GYROSCOPE\n"
            f"{'='*24}\n"
            f"gx: {gyr[0]:>8.4f}\n"
            f"gy: {gyr[1]:>8.4f}\n"
            f"\n"
            f"TEMPERATURE\n"
            f"{'='*24}\n"
            f"temp: {temp_str}"
        )

    def run(self):
        self.ani = FuncAnimation(
            self.fig, self.update,
            interval=100,
            cache_frame_data=False,
        )
        plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    packet_queue: deque = deque(maxlen=args.buf)

    receiver = UdpReceiver(
        port=args.port,
        node_filter=args.node,
        packet_queue=packet_queue,
        csv_path=args.csv,
        timeout=args.timeout,
    )
    receiver.start()

    try:
        viz = DRIFTVisualizer(packet_queue)
        viz.run()
    finally:
        receiver.stop()
        receiver.join(timeout=2.0)
        print("Done.")


if __name__ == "__main__":
    main()