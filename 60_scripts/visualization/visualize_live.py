#!/usr/bin/env python3
"""
visualize_live.py  –  Real-time DRIFT visualizer fed by UDP/JSON packets
from ESP32 IMU nodes (replaces synthetic get_telemetry()).

Packet format expected (JSON over UDP):
  {"node": 1, "ts": 123456, "ax": 0.1, "ay": -0.2, "az": 9.8,
                             "gx": 0.01, "gy": -0.01, "gz": 0.002}

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
                "node", "ts", "ax", "ay", "az", "gx", "gy", "gz", "raw_json",
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
    "font.family":    "Franklin Gothic Medium",
    "text.color":     "#E0E0E0",
    "axes.labelcolor":"#E0E0E0",
    "xtick.color":    "#B0B0B0",
    "ytick.color":    "#B0B0B0",
    "axes.facecolor": "#2B2B2B",
    "figure.facecolor":"#1E1E1E",
})


class DRIFTVisualizer:
    def __init__(self, packet_queue: deque):
        self.queue      = packet_queue
        self.integrator = Integrator()

        # ── Layout ──────────────────────────────────────────────────────────
        self.fig = plt.figure(figsize=(16, 9), facecolor="#1E1E1E")
        self.gs  = self.fig.add_gridspec(3, 4, wspace=0.3, hspace=0.4)

        self.ax_traj    = self.fig.add_subplot(self.gs[:, 0:2])
        self.ax_accel   = self.fig.add_subplot(self.gs[0, 2])
        self.ax_gyro    = self.fig.add_subplot(self.gs[1, 2])
        self.ax_vel     = self.fig.add_subplot(self.gs[2, 2])
        self.ax_metrics = self.fig.add_subplot(self.gs[:, 3])
        self.ax_metrics.axis("off")

        # ── Trajectory paths ────────────────────────────────────────────────
        self.path  = {"x": [], "y": []}

        # ── Rolling history buffers (last N samples) ─────────────────────
        self.buf_len = 100
        self.a_buf   = np.zeros((self.buf_len, 3))   # accelerometer
        self.g_buf   = np.zeros((self.buf_len, 3))   # gyroscope
        self.v_buf   = np.zeros((self.buf_len, 3))   # integrated velocity

        # ── Counters ─────────────────────────────────────────────────────
        self.total_dist  = 0.0
        self.pkt_count   = 0
        self.last_packet: dict = {}

        # ── Status line in window title ──────────────────────────────────
        self.fig.canvas.manager.set_window_title("DRIFT – Waiting for packets…")

    # ── Drain the queue and integrate all waiting packets ─────────────────

    def _drain_queue(self):
        """
        Process every packet that arrived since the last animation frame.
        Returns the most-recent parsed state, or None if the queue is empty.
        """
        latest = None
        while self.queue:
            pkt = self.queue.popleft()
            self.pkt_count += 1

            ax = float(pkt.get("ax") or 0)
            ay = float(pkt.get("ay") or 0)
            az = float(pkt.get("az") or 0)
            gx = float(pkt.get("gx") or 0)
            gy = float(pkt.get("gy") or 0)
            gz = float(pkt.get("gz") or 0)
            ts = float(pkt.get("ts") or 0)

            pos, vel = self.integrator.update(ax, ay, az, ts)

            # Accumulate distance
            if self.path["x"]:
                dx = pos[0] - self.path["x"][-1]
                dy = pos[1] - self.path["y"][-1]
                self.total_dist += np.sqrt(dx*dx + dy*dy)

            self.path["x"].append(pos[0])
            self.path["y"].append(pos[1])

            # Roll buffers
            for buf, vec in ((self.a_buf, [ax, ay, az]),
                             (self.g_buf, [gx, gy, gz]),
                             (self.v_buf, vel)):
                buf[:] = np.roll(buf, -1, axis=0)
                buf[-1] = vec

            latest = dict(pos=pos, vel=vel,
                          accel=[ax, ay, az], gyro=[gx, gy, gz],
                          node=pkt.get("node"), ts=ts,
                          source=pkt.get("_source_ip", "?"))
        return latest

    # ── FuncAnimation callback ─────────────────────────────────────────────

    def update(self, _frame):
        state = self._drain_queue()

        if state is None:
            # No new data – update title and return without redrawing
            self.fig.canvas.manager.set_window_title(
                f"DRIFT – waiting… ({self.pkt_count} received)")
            return

        pos   = state["pos"]
        vel   = state["vel"]
        accel = state["accel"]
        gyro  = state["gyro"]
        node  = state["node"]
        src   = state["source"]

        self.fig.canvas.manager.set_window_title(
            f"DRIFT – node={node}  src={src}  pkts={self.pkt_count}")

        # ── Trajectory ────────────────────────────────────────────────────
        self.ax_traj.cla()
        self.ax_traj.set_title("2D Trajectory (X-Y Plane)", color="#00FFCC", fontsize=14)
        self.ax_traj.plot(self.path["x"], self.path["y"],
                          color="#00FFCC", linewidth=2, label=f"Node {node}")
        # Mark start
        if len(self.path["x"]) >= 1:
            self.ax_traj.plot(self.path["x"][0], self.path["y"][0],
                              "o", color="#FFAA00", markersize=6, label="Start")
        self.ax_traj.set_xlabel("X (m)")
        self.ax_traj.set_ylabel("Y (m)")
        self.ax_traj.legend(facecolor="#1E1E1E", edgecolor="#444444", loc="lower right")
        self.ax_traj.grid(True, color="#333333")

        # ── Telemetry subplots ────────────────────────────────────────────
        plots = [
            (self.ax_accel, self.a_buf,
             f"Accel (m/s²) | az={accel[2]:.2f}",
             ["#FF5555", "#55FF55", "#5555FF"],
             ["ax", "ay", "az"]),
            (self.ax_gyro, self.g_buf,
             f"Gyro (rad/s) | gz={gyro[2]:.3f}",
             ["#FFAA00", "#AAFF00", "#00AAFF"],
             ["gx", "gy", "gz"]),
            (self.ax_vel, self.v_buf,
             f"Velocity (m/s) | |v|={np.linalg.norm(vel):.2f}",
             ["#FF00FF", "#00FFFF", "#FFFF00"],
             ["vx", "vy", "vz"]),
        ]

        for ax, data, title, colors, labels in plots:
            ax.cla()
            ax.set_title(title, loc="left", fontsize=10)
            for i in range(3):
                ax.plot(data[:, i], color=colors[i], alpha=0.8, label=labels[i])
            ax.legend(facecolor="#1E1E1E", edgecolor="#333333",
                      fontsize=7, loc="upper left")
            ax.grid(True, color="#333333", alpha=0.5)

        # ── Metrics panel ─────────────────────────────────────────────────
        speed = np.linalg.norm(vel)
        self.ax_metrics.cla()
        self.ax_metrics.axis("off")
        self.ax_metrics.text(
            0.05, 0.95,
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
                f"py: {pos[1]:>8.3f} m\n"
                f"pz: {pos[2]:>8.3f} m\n\n"
                f"VELOCITY\n"
                f"{'='*25}\n"
                f"vx: {vel[0]:>8.3f} m/s\n"
                f"vy: {vel[1]:>8.3f} m/s\n"
                f"vz: {vel[2]:>8.3f} m/s\n"
                f"|v|:{speed:>8.3f} m/s\n\n"
                f"ACCELEROMETER\n"
                f"{'='*25}\n"
                f"ax: {accel[0]:>8.3f}\n"
                f"ay: {accel[1]:>8.3f}\n"
                f"az: {accel[2]:>8.3f}\n\n"
                f"GYROSCOPE\n"
                f"{'='*25}\n"
                f"gx: {gyro[0]:>8.4f}\n"
                f"gy: {gyro[1]:>8.4f}\n"
                f"gz: {gyro[2]:>8.4f}"
            ),
            transform=self.ax_metrics.transAxes,
            va="top", fontsize=10, color="#00FFCC",
            family="Franklin Gothic Medium",
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