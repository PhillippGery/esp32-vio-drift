import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Global Styling
plt.rcParams.update({
    'font.family': 'Franklin Gothic Medium',
    'text.color': '#E0E0E0',
    'axes.labelcolor': '#E0E0E0',
    'xtick.color': '#B0B0B0',
    'ytick.color': '#B0B0B0',
    'axes.facecolor': '#2B2B2B',
    'figure.facecolor': '#1E1E1E'
})


class DRIFTVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(16, 9), facecolor='#1E1E1E')
        self.gs = self.fig.add_gridspec(3, 4, wspace=0.3, hspace=0.4)

        # 1. 2D Trajectory Plot (Ignoring Z)
        self.ax_traj = self.fig.add_subplot(self.gs[:, 0:2])
        self.path_data = {"x": [], "y": []}
        self.gt_path = {"x": [], "y": []}

        # 2. Telemetry Graphs
        self.ax_vel = self.fig.add_subplot(self.gs[0, 2])
        self.ax_ori = self.fig.add_subplot(self.gs[1, 2])
        self.ax_bias = self.fig.add_subplot(self.gs[2, 2])

        # 3. Metrics & State Panel
        self.ax_metrics = self.fig.add_subplot(self.gs[:, 3])
        self.ax_metrics.axis('off')

        # Buffers for history
        self.buf_len = 100
        self.v_buf = np.zeros((self.buf_len, 3))
        self.o_buf = np.zeros((self.buf_len, 3))
        self.b_buf = np.zeros((self.buf_len, 3))

        self.total_dist = 0.0
        self.count = 0

    def get_telemetry(self):
        """Generates synthetic 12-DOF data for simulation."""
        t = self.count * 0.1
        # Target: Circular trajectory (1m diameter) [cite: 39]
        gt_x, gt_y = 0.5 * np.cos(t), 0.5 * np.sin(t)

        # Simulated VIO state with accumulated drift [cite: 25]
        drift = self.count * 0.003
        px, py, pz = gt_x + drift, gt_y + drift, 0.1 * t
        vx, vy, vz = -0.5 * np.sin(t), 0.5 * np.cos(t), 0.1
        r, p, y = 0.02 * np.sin(t), 0.02 * np.cos(t), t % (2 * np.pi)
        bx, by, bz = 0.001, -0.001, 0.0005

        if self.count > 0:
            self.total_dist += np.sqrt((px - self.path_data["x"][-1]) ** 2 + (py - self.path_data["y"][-1]) ** 2)

        self.count += 1
        state = [px, py, pz, vx, vy, vz, r, p, y, bx, by, bz]
        return state, [gt_x, gt_y]

    def update(self, frame):
        state, gt = self.get_telemetry()
        pos, vel, ori, bias = state[0:3], state[3:6], state[6:9], state[9:12]

        # Update Trajectory
        self.path_data["x"].append(pos[0]);
        self.path_data["y"].append(pos[1])
        self.gt_path["x"].append(gt[0]);
        self.gt_path["y"].append(gt[1])

        self.ax_traj.cla()
        self.ax_traj.set_title("2D Trajectory (X-Y Plane)", color='#00FFCC', fontsize=14)
        self.ax_traj.plot(self.gt_path["x"], self.gt_path["y"], color='#444444', linestyle='--', label='Reference')
        self.ax_traj.plot(self.path_data["x"], self.path_data["y"], color='#00FFCC', linewidth=2, label='VIO Node 1')
        self.ax_traj.set_xlabel("X (m)");
        self.ax_traj.set_ylabel("Y (m)")
        self.ax_traj.legend(facecolor='#1E1E1E', edgecolor='#444444', loc='lower right')
        self.ax_traj.grid(True, color='#333333')

        # Update History Buffers
        for b, v in zip([self.v_buf, self.o_buf, self.b_buf], [vel, ori, bias]):
            b[:] = np.roll(b, -1, axis=0);
            b[-1] = v

        # Subplot Refresh
        plots = [
            (self.ax_vel, self.v_buf, f"Velocity (m/s) | v: {np.linalg.norm(vel):.2f}",
             ['#FF5555', '#55FF55', '#5555FF']),
            (self.ax_ori, self.o_buf, f"Euler Angles (rad) | y: {ori[2]:.2f}", ['#FFAA00', '#AAFF00', '#00AAFF']),
            (self.ax_bias, self.b_buf, f"Gyro Bias (rad/s) | bz: {bias[2]:.4f}", ['#FF00FF', '#00FFFF', '#FFFF00'])
        ]

        for ax, data, title, colors in plots:
            ax.cla()
            ax.set_title(title, loc='left', fontsize=10)
            for i in range(3):
                ax.plot(data[:, i], color=colors[i], alpha=0.8)
            ax.grid(True, color='#333333', alpha=0.5)

        # Drift & State Calculation [cite: 42, 43]
        end_error = np.sqrt((pos[0] - gt[0]) ** 2 + (pos[1] - gt[1]) ** 2)
        speed = np.linalg.norm(vel)
        drift_rate = (end_error / self.total_dist) * 100 if self.total_dist > 0 else 0

        self.ax_metrics.cla()
        self.ax_metrics.axis('off')

        telemetry_text = (
            f"DRIFT EVALUATION\n"
            f"{'=' * 25}\n"
            f"Endpoint Error: {end_error:.4f} m\n"
            f"Drift Rate:     {drift_rate:.2f} %\n"
            f"Current Speed:  {speed:.2f} m/s\n\n"
            f"12-DOF STATE VECTOR\n"
            f"{'=' * 25}\n"
            f"px: {pos[0]:>7.3f} | py: {pos[1]:>7.3f}\n"
            f"pz: {pos[2]:>7.3f} (ignored)\n\n"
            f"vx: {vel[0]:>7.3f} | vy: {vel[1]:>7.3f}\n"
            f"vz: {vel[2]:>7.3f}\n\n"
            f"roll:  {ori[0]:>5.3f} | pitch: {ori[1]:>5.3f}\n"
            f"yaw:   {ori[2]:>5.3f}\n\n"
            f"bx: {bias[0]:>8.4f} | by: {bias[1]:>8.4f}\n"
            f"bz: {bias[2]:>8.4f}"
        )
        self.ax_metrics.text(0.05, 0.95, telemetry_text, transform=self.ax_metrics.transAxes,
                             va='top', family='Franklin Gothic Medium', fontsize=11, color='#00FFCC')

    def run(self):
        self.ani = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.show()


if __name__ == "__main__":
    DRIFTVisualizer().run()