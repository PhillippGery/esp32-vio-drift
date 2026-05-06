#!/usr/bin/env python3
"""
visualize_live.py  –  Real-time DRIFT visualizer fed by UDP/JSON packets
from ESP32 IMU nodes (replaces synthetic get_telemetry()).

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

        # Socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)
        self.sock.bind(("0.0.0.0", self.port))

        # Optional CSV log
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

            # Annotate with metadata and push
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

        # Cleanup
        if self._csv_file:
            self._csv_file.close()
        self.sock.close()
        print(f"[UDP] Stopped. Total received: {self.received}")


# ---------------------------------------------------------------------------
# Minimal dead-reckoning integrator
# ---------------------------------------------------------------------------

class Integrator:
    """
    Very simple trapezoidal integration of accelerometer data → velocity → position.
    Zeroes out velocity if the norm is below a small threshold (crude zero-velocity
    update to limit runaway drift when the node is still).

    For a production system, replace this with your VIO / EKF output fed directly
    through the UDP packet fields.
    """
    ZUPT_THRESHOLD = 0.05   # m/s  – velocity clamped to 0 below this norm
    GRAVITY        = 9.81   # m/s²

    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self._last_ts: float = None   # seconds

    def update(self, ax, ay, az, ts_ms: float):
        """
        ts_ms : device timestamp in milliseconds (wraps at ~49 days for uint32).
        Returns updated (pos, vel).
        """
        ts_s = ts_ms / 1000.0
        if self._last_ts is None:
            self._last_ts = ts_s
            return self.pos.copy(), self.vel.copy()

        dt = ts_s - self._last_ts
        # Guard against clock wrap-around or duplicate packets
        if dt <= 0 or dt > 1.0:
            self._last_ts = ts_s
            return self.pos.copy(), self.vel.copy()

        self._last_ts = ts_s

        # Remove gravity from Z and integrate
        accel = np.array([ax, ay, az - self.GRAVITY])
        self.vel += accel * dt
        if np.linalg.norm(self.vel) < self.ZUPT_THRESHOLD:
            self.vel[:] = 0.0
        self.pos += self.vel * dt

        return self.pos.copy(), self.vel.copy()


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

plt.rcParams.update({
    # "font.family":     "Franklin Gothic Medium",
    "text.color":      "#E0E0E0",
    "axes.labelcolor": "#E0E0E0",
    "xtick.color":     "#B0B0B0",
    "ytick.color":     "#B0B0B0",
    "axes.facecolor":  "#2B2B2B",
    "figure.facecolor":"#1E1E1E",
})


class DRIFTVisualizer:
    def __init__(self, packet_queue: deque):
        self.queue      = packet_queue
        self.integrator = Integrator()

        # ── Layout: 4 rows × 4 cols ──────────────────────────────────────────
        #   col 0-1 : full-height trajectory
        #   col 2   : row 0 accel | row 1 gyro | row 2 velocity | row 3 temp
        #   col 3   : full-height metrics text panel
        self.fig = plt.figure(figsize=(16, 9), facecolor="#1E1E1E")
        self.gs  = self.fig.add_gridspec(4, 4, wspace=0.32, hspace=0.45)

        self.ax_traj    = self.fig.add_subplot(self.gs[:, 0:2])
        self.ax_accel   = self.fig.add_subplot(self.gs[0, 2])
        self.ax_gyro    = self.fig.add_subplot(self.gs[1, 2])
        self.ax_vel     = self.fig.add_subplot(self.gs[2, 2])
        self.ax_temp    = self.fig.add_subplot(self.gs[3, 2])
        self.ax_metrics = self.fig.add_subplot(self.gs[:, 3])
        self.ax_metrics.axis("off")

        # ── Trajectory path ──────────────────────────────────────────────────
        self.path = {"x": [], "y": []}

        # ── Rolling history buffers ──────────────────────────────────────────
        # Deques grow from empty, so every plotted sample is real data.
        # Each entry holds only the X/Y components (Z removed).
        self.buf_len = 100
        self.a_buf = deque(maxlen=self.buf_len)   # [ax, ay]
        self.g_buf = deque(maxlen=self.buf_len)   # [gx, gy]
        self.v_buf = deque(maxlen=self.buf_len)   # [vx, vy]
        self.t_buf = deque(maxlen=self.buf_len)   # float °C (skips None entries)

        # ── Counters / last-known values ─────────────────────────────────────
        self.total_dist = 0.0
        self.pkt_count  = 0
        self.last_temp: float | None = None

        # ── Window title ─────────────────────────────────────────────────────
        self.fig.canvas.manager.set_window_title("DRIFT – Waiting for packets…")

    # ── Drain the queue and integrate all waiting packets ─────────────────────

    def _drain_queue(self):
        """
        Process every packet that arrived since the last animation frame.
        Returns the most-recent parsed state dict, or None if the queue was empty.
        """
        latest = None
        while self.queue:
            pkt = self.queue.popleft()
            self.pkt_count += 1

            # Explicit None-check: genuine 0.0 readings are preserved, not coerced.
            def _f(key):
                v = pkt.get(key)
                return float(v) if v is not None else 0.0

            ax_v = _f("ax")
            ay_v = _f("ay")
            az_v = _f("az")
            gx_v = _f("gx")
            gy_v = _f("gy")
            # gz kept for integrator input but not buffered for display
            gz_v = _f("gz")   # noqa: F841
            ts_v = _f("ts")

            raw_temp = pkt.get("temp")
            if raw_temp is not None:
                self.last_temp = float(raw_temp)

            pos, vel = self.integrator.update(ax_v, ay_v, az_v, ts_v)

            # Accumulate XY distance
            if self.path["x"]:
                dx = pos[0] - self.path["x"][-1]
                dy = pos[1] - self.path["y"][-1]
                self.total_dist += np.sqrt(dx * dx + dy * dy)

            self.path["x"].append(pos[0])
            self.path["y"].append(pos[1])

            # Append X/Y components only; temperature only when present
            self.a_buf.append([ax_v, ay_v])
            self.g_buf.append([gx_v, gy_v])
            self.v_buf.append([vel[0], vel[1]])
            if self.last_temp is not None:
                self.t_buf.append(self.last_temp)

            latest = dict(
                pos=pos, vel=vel,
                accel=[ax_v, ay_v],
                gyro=[gx_v, gy_v],
                temp=self.last_temp,
                node=pkt.get("node"),
                ts=ts_v,
                source=pkt.get("_source_ip", "?"),
            )
        return latest

    # ── Helper: unpack a deque of 2-element rows into two numpy arrays ─────────

    @staticmethod
    def _cols(buf):
        """Return (col0_array, col1_array) from a deque of [x, y] rows."""
        if not buf:
            return np.array([]), np.array([])
        arr = np.array(buf)      # shape (N, 2)
        return arr[:, 0], arr[:, 1]

    # ── FuncAnimation callback ─────────────────────────────────────────────────

    def update(self, _frame):
        state = self._drain_queue()

        if state is None:
            self.fig.canvas.manager.set_window_title(
                f"DRIFT – waiting… ({self.pkt_count} received)")
            return

        pos   = state["pos"]
        vel   = state["vel"]
        accel = state["accel"]   # [ax, ay]
        gyro  = state["gyro"]    # [gx, gy]
        temp  = state["temp"]
        node  = state["node"]
        src   = state["source"]
        speed = float(np.linalg.norm(vel[:2]))  # XY speed

        self.fig.canvas.manager.set_window_title(
            f"DRIFT – node={node}  src={src}  pkts={self.pkt_count}")

        # ── Trajectory ─────────────────────────────────────────────────────
        self.ax_traj.cla()
        self.ax_traj.set_facecolor("#2B2B2B")
        self.ax_traj.set_title("2D Trajectory (X-Y Plane)", color="#00FFCC", fontsize=14)
        self.ax_traj.plot(self.path["x"], self.path["y"],
                          color="#00FFCC", linewidth=2, label=f"Node {node}")
        if self.path["x"]:
            self.ax_traj.plot(self.path["x"][0], self.path["y"][0],
                              "o", color="#FFAA00", markersize=6, label="Start")
        self.ax_traj.set_xlabel("X (m)")
        self.ax_traj.set_ylabel("Y (m)")
        self.ax_traj.legend(facecolor="#1E1E1E", edgecolor="#444444", loc="lower right")
        self.ax_traj.grid(True, color="#333333")

        # ── Telemetry subplots (X & Y only) ────────────────────────────────
        xy_plots = [
            (self.ax_accel, self.a_buf,
             f"Accel (m/s²)  ax={accel[0]:.2f}  ay={accel[1]:.2f}",
             ["#FF5555", "#55FF55"], ["ax", "ay"]),

            (self.ax_gyro, self.g_buf,
             f"Gyro (rad/s)  gx={gyro[0]:.3f}  gy={gyro[1]:.3f}",
             ["#FFAA00", "#AAFF00"], ["gx", "gy"]),

            (self.ax_vel, self.v_buf,
             f"Velocity (m/s)  |v|={speed:.2f}",
             ["#FF00FF", "#00FFFF"], ["vx", "vy"]),
        ]

        for subplot_ax, buf, title, colors, labels in xy_plots:
            subplot_ax.cla()
            subplot_ax.set_facecolor("#2B2B2B")
            subplot_ax.set_title(title, loc="left", fontsize=9)
            c0, c1 = self._cols(buf)
            if c0.size:
                subplot_ax.plot(c0, color=colors[0], alpha=0.9, label=labels[0])
                subplot_ax.plot(c1, color=colors[1], alpha=0.9, label=labels[1])
            subplot_ax.legend(facecolor="#1E1E1E", edgecolor="#333333",
                              fontsize=7, loc="upper left")
            subplot_ax.grid(True, color="#333333", alpha=0.5)

        # ── Temperature subplot ─────────────────────────────────────────────
        self.ax_temp.cla()
        self.ax_temp.set_facecolor("#2B2B2B")
        temp_str = f"{temp:.2f} °C" if temp is not None else "N/A"
        self.ax_temp.set_title(f"Temperature  {temp_str}", loc="left", fontsize=9)
        if self.t_buf:
            self.ax_temp.plot(list(self.t_buf), color="#FF8C00",
                              alpha=0.9, linewidth=1.5, label="temp (°C)")
            self.ax_temp.legend(facecolor="#1E1E1E", edgecolor="#333333",
                                fontsize=7, loc="upper left")
        self.ax_temp.set_ylabel("°C", fontsize=8)
        self.ax_temp.grid(True, color="#333333", alpha=0.5)

        # ── Metrics panel ───────────────────────────────────────────────────
        self.ax_metrics.cla()
        self.ax_metrics.axis("off")
        self.ax_metrics.text(
            0.05, 0.97,
            (
                f"LIVE TELEMETRY\n"
                f"{'='*25}\n"
                f"Node:        {node}\n"
                f"Source:      {src}\n"
                f"Packets:     {self.pkt_count}\n"
                f"Dist (XY):   {self.total_dist:.3f} m\n\n"
                f"POSITION\n"
                f"{'='*25}\n"
                f"px: {pos[0]:>8.3f} m\n"
                f"py: {pos[1]:>8.3f} m\n\n"
                f"VELOCITY\n"
                f"{'='*25}\n"
                f"vx: {vel[0]:>8.3f} m/s\n"
                f"vy: {vel[1]:>8.3f} m/s\n"
                f"|v|:{speed:>8.3f} m/s\n\n"
                f"ACCELEROMETER\n"
                f"{'='*25}\n"
                f"ax: {accel[0]:>8.3f}\n"
                f"ay: {accel[1]:>8.3f}\n\n"
                f"GYROSCOPE\n"
                f"{'='*25}\n"
                f"gx: {gyro[0]:>8.4f}\n"
                f"gy: {gyro[1]:>8.4f}\n\n"
                f"TEMPERATURE\n"
                f"{'='*25}\n"
                f"temp: {temp_str}"
            ),
            transform=self.ax_metrics.transAxes,
            va="top", fontsize=10, color="#00FFCC",
            # family="Franklin Gothic Medium",
        )

    def run(self):
        self.ani = FuncAnimation(
            self.fig, self.update,
            interval=100,           # refresh every 100 ms
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    # Shared thread-safe FIFO (bounded to avoid unbounded memory on fast senders)
    packet_queue: deque = deque(maxlen=args.buf)

    # Start UDP receiver in background
    receiver = UdpReceiver(
        port=args.port,
        node_filter=args.node,
        packet_queue=packet_queue,
        csv_path=args.csv,
        timeout=args.timeout,
    )
    receiver.start()

    # Run visualizer on main thread (matplotlib requirement)
    try:
        viz = DRIFTVisualizer(packet_queue)
        viz.run()
    finally:
        receiver.stop()
        receiver.join(timeout=2.0)
        print("Done.")


if __name__ == "__main__":
    main()