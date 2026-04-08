import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# Pick font
plt.rcParams['font.family'] = 'Franklin Gothic Medium'


class DRIFTVisualizer:
    def __init__(self):
        # Setup Figure with a layout for 3D trajectory and 3 telemetry subplots
        self.fig = plt.figure(figsize=(15, 10))
        self.gs = self.fig.add_gridspec(3, 4)

        # 1. 3D Trajectory Plot (Large)
        self.ax_3d = self.fig.add_subplot(self.gs[:, 0:2], projection='3d')
        self.path_data = {"x": [], "y": [], "z": []}
        self.gt_path = {"x": [], "y": [], "z": []}  # For drift calculation

        # 2. Velocity Plot
        self.ax_vel = self.fig.add_subplot(self.gs[0, 2])

        # 3. Orientation Plot
        self.ax_ori = self.fig.add_subplot(self.gs[1, 2])

        # 4. Gyro Bias Plot
        self.ax_bias = self.fig.add_subplot(self.gs[2, 2])

        # 5. Drift Metrics Panel
        self.ax_stats = self.fig.add_subplot(self.gs[:, 3])
        self.ax_stats.axis('off')

        # Data buffers
        self.history_len = 100
        self.vel_buffer = np.zeros((self.history_len, 3))
        self.ori_buffer = np.zeros((self.history_len, 3))
        self.bias_buffer = np.zeros((self.history_len, 3))

        self.total_distance = 0.0
        self.count = 0

    def generate_synthetic_data(self):
        """Generates synthetic 12-DOF state and a ground-truth for drift eval."""
        t = self.count * 0.1

        # Ground Truth (Perfect Circle)
        gt_x, gt_y, gt_z = np.cos(t), np.sin(t), 0.0

        # Estimated Position (Adding intentional drift over time)
        drift_factor = self.count * 0.005
        px, py, pz = gt_x + drift_factor, gt_y + (drift_factor * 0.5), gt_z

        # State Vector Components
        vx, vy, vz = -np.sin(t), np.cos(t), 0.0
        roll, pitch, yaw = 0.05 * np.sin(t), 0.05 * np.cos(t), t % (2 * np.pi)
        bx, by, bz = 0.01, -0.005, 0.002  # Static biases

        if self.count > 0:
            prev_p = [self.path_data["x"][-1], self.path_data["y"][-1], self.path_data["z"][-1]]
            self.total_distance += np.linalg.norm(np.array([px, py, pz]) - np.array(prev_p))

        self.count += 1
        return [px, py, pz, vx, vy, vz, roll, pitch, yaw, bx, by, bz], [gt_x, gt_y, gt_z]

    def update(self, frame):
        state, gt = self.generate_synthetic_data()
        p, v, ori, bias = state[0:3], state[3:6], state[6:9], state[9:12]

        # Update Paths
        self.path_data["x"].append(p[0]);
        self.path_data["y"].append(p[1]);
        self.path_data["z"].append(p[2])
        self.gt_path["x"].append(gt[0]);
        self.gt_path["y"].append(gt[1]);
        self.gt_path["z"].append(gt[2])

        # 3D Plot Update
        self.ax_3d.cla()
        self.ax_3d.plot(self.gt_path["x"], self.gt_path["y"], self.gt_path["z"], 'g--', alpha=0.3, label='Ground Truth')
        self.ax_3d.plot(self.path_data["x"], self.path_data["y"], self.path_data["z"], 'b-', label='VIO Estimate')
        self.ax_3d.set_title("VIO Real-time Trajectory (Node 1)")
        self.ax_3d.legend(loc='upper left', fontsize='small')

        # Update Time-Series Buffers
        for buf, val in zip([self.vel_buffer, self.ori_buffer, self.bias_buffer], [v, ori, bias]):
            buf[:] = np.roll(buf, -1, axis=0)
            buf[-1] = val

        # Velocity Graph
        self.ax_vel.cla()
        self.ax_vel.set_title(f"Velocity (m/s) | Current: [{v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}]")
        self.ax_vel.plot(self.vel_buffer)

        # Orientation Euler Graph
        self.ax_ori.cla()
        self.ax_ori.set_title(f"Euler Angles (rad) | Current: [{ori[0]:.2f}, {ori[1]:.2f}, {ori[2]:.2f}]")
        self.ax_ori.plot(self.ori_buffer)

        # Gyro Bias Graph
        self.ax_bias.cla()
        self.ax_bias.set_title(f"Gyro Bias (rad/s) | Current: [{bias[0]:.3f}, {bias[1]:.3f}, {bias[2]:.3f}]")
        self.ax_bias.plot(self.bias_buffer)

        # Drift Estimation Section
        endpoint_error = np.linalg.norm(np.array(p) - np.array(gt))
        speed = np.linalg.norm(v)
        drift_rate = (endpoint_error / self.total_distance) * 100 if self.total_distance > 0 else 0

        self.ax_stats.cla()
        self.ax_stats.axis('off')
        stats_text = (
            f"DRIFT ESTIMATION\n"
            f"--------------------------\n"
            f"Endpoint Error: {endpoint_error:.4f} m\n"
            f"Drift Rate:     {drift_rate:.2f} %\n"
            f"Current Speed:  {speed:.2f} m/s\n\n"
            f"STATE VARIABLES\n"
            f"--------------------------\n"
            f"px: {p[0]:.3f} | py: {p[1]:.3f} | pz: {p[2]:.3f}\n"
            f"vx: {v[0]:.3f} | vy: {v[1]:.3f} | vz: {v[2]:.3f}\n"
            f"roll: {ori[0]:.3f} | pitch: {ori[1]:.3f} | yaw: {ori[2]:.3f}\n"
            f"bx: {bias[0]:.4f} | by: {bias[1]:.4f} | bz: {bias[2]:.4f}"
        )
        self.ax_stats.text(0, 1, stats_text, transform=self.ax_stats.transAxes,
                           verticalalignment='top', family='Franklin Gothic Medium', fontsize=11)

    def animate(self):
        ani = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    viz = DRIFTVisualizer()
    viz.animate()