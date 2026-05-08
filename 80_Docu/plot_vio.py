import numpy as np
import matplotlib.pyplot as plt

# ─── 1. SIMULATION PARAMETERS ────────────────────────────────────────────────
time_seconds = 10  # Reduced to 10 seconds per your instruction
samples = 1000     # Scaled down samples to match the shorter timeframe
time = np.linspace(0, time_seconds, samples)

# ─── 2. MATH MODELS ─────────────────────────────────────────────────────────
# IMU Only: Drift grows quadratically due to double integration of noise
imu_drift_factor = 0.15
imu_only_error = imu_drift_factor * (time ** 2)

# VIO + EKF Fused: Error grows, but resets at every camera update
update_interval = 0.5 # Camera provides a metric update every 2.5 seconds
time_since_last_update = time % update_interval

# The EKF drops the error back to the camera's measurement noise floor (0.1m)
measurement_noise_floor = 0.1 
ekf_error = measurement_noise_floor + (imu_drift_factor * (time_since_last_update ** 2))

# ─── 3. PLOT CONFIGURATION ──────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
fig.suptitle('Odometry Error: IMU Dead-Reckoning vs. EKF VIO Fusion', fontsize=16, fontweight='bold')

# Subplot 1: IMU Only
ax1.plot(time, imu_only_error, color='#e74c3c', linewidth=2.5, label='IMU Dead-Reckoning')
ax1.set_ylabel('Positional Error (m)', fontsize=12, fontweight='bold')
ax1.set_title('Node 4: Uncorrected MPU-6050 (Drift to Infinity)', fontsize=12)
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.fill_between(time, imu_only_error, color='#e74c3c', alpha=0.1)
ax1.legend(loc='upper left')

# Subplot 2: EKF VIO
ax2.plot(time, ekf_error, color='#2ecc71', linewidth=2.5, label='Planar EKF (IMU + VIO)')
ax2.set_ylabel('Positional Error (m)', fontsize=12, fontweight='bold')
ax2.set_xlabel('Time (Seconds)', fontsize=12, fontweight='bold')
ax2.set_title('Node 1: IPM Vision Pipeline (Sawtooth Variance Correction)', fontsize=12)
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.fill_between(time, ekf_error, color='#2ecc71', alpha=0.2)

# Add dashed lines to show the exact moments the camera fires
for t in np.arange(update_interval, time_seconds, update_interval):
    ax2.axvline(x=t, color='black', linestyle='--', alpha=0.3, linewidth=1)

ax2.legend(loc='upper left')

# Set specific Y-limits to emphasize the scale difference
ax1.set_ylim(0, max(imu_only_error) * 1.1)
ax2.set_ylim(0, max(ekf_error) * 2.0) 

# ─── 4. RENDER AND SAVE ─────────────────────────────────────────────────────
plt.tight_layout()
plt.savefig('vio_ekf_comparison.png', dpi=300, bbox_inches='tight')
print("Plot saved as 'vio_ekf_comparison.png'")
plt.show()