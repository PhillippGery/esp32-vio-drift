

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ─── 1. SIMULATE THE 3D GROUND PLANE (Z is forward, X is lateral) ──────────
# Define a physical grid on the floor (e.g., 10m forward, 4m wide)
Z_grid = np.linspace(1, 10, 10)  # Depth (1m to 10m)
X_grid = np.linspace(-2, 2, 9)   # Width (-2m to 2m)

# Camera intrinsic parameters (Simulated)
f = 800  # Focal length
cx, cy = 320, 240  # Principal point (640x480 image)
h = 0.5  # Camera height from ground (meters)

# ─── 2. SET UP THE PLOT ─────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle('Inverse Perspective Mapping (IPM) for VIO Scale Ambiguity Resolution', fontsize=16, fontweight='bold')

# ─── 3. LEFT PLOT: RAW CAMERA VIEW (Perspective) ────────────────────────────
ax1.set_title('Camera Image Plane (Pixels)', fontsize=14, fontweight='bold')
ax1.set_xlim(0, 640)
ax1.set_ylim(480, 0) # Inverted Y for image coordinates
ax1.set_facecolor('#ecf0f1')

# Draw the perspective grid
for z in Z_grid:
    u_line = []
    v_line = []
    for x in X_grid:
        u = f * (x / z) + cx
        v = f * (h / z) + cy
        u_line.append(u)
        v_line.append(v)
    ax1.plot(u_line, v_line, color='#7f8c8d', alpha=0.6)

for x in X_grid:
    u_line = []
    v_line = []
    for z in Z_grid:
        u = f * (x / z) + cx
        v = f * (h / z) + cy
        u_line.append(u)
        v_line.append(v)
    ax1.plot(u_line, v_line, color='#7f8c8d', alpha=0.6)

# Simulate a FAST-9 corner moving forward (distorted in pixel space)
p1_z, p2_z = 3.0, 4.0
p_x = 0.5
u1, v1 = f * (p_x / p1_z) + cx, f * (h / p1_z) + cy
u2, v2 = f * (p_x / p2_z) + cx, f * (h / p2_z) + cy

ax1.plot([u1], [v1], 'ro', markersize=8, label='Feature (t)')
ax1.plot([u2], [v2], 'bo', markersize=8, label='Feature (t+1)')
ax1.annotate('', xy=(u2, v2), xytext=(u1, v1),
             arrowprops=dict(arrowstyle='->', color='black', lw=2))
ax1.set_xlabel('Pixel U', fontsize=12)
ax1.set_ylabel('Pixel V', fontsize=12)
ax1.legend(loc='upper right')

# ─── 4. RIGHT PLOT: IPM BIRD'S EYE VIEW (Metric) ────────────────────────────
ax2.set_title('IPM Ground Plane (Metric Meters)', fontsize=14, fontweight='bold')
ax2.set_xlim(-2.5, 2.5)
ax2.set_ylim(0, 11)
ax2.set_facecolor('#ecf0f1')

# Draw the metric grid (perfectly square)
for z in Z_grid:
    ax2.hlines(z, xmin=X_grid[0], xmax=X_grid[-1], color='#34495e', alpha=0.4)
for x in X_grid:
    ax2.vlines(x, ymin=Z_grid[0], ymax=Z_grid[-1], color='#34495e', alpha=0.4)

# Show the exact same feature movement, but now in metric scale
ax2.plot([p_x], [p1_z], 'ro', markersize=8, label='Feature (t)')
ax2.plot([p_x], [p2_z], 'bo', markersize=8, label='Feature (t+1)')
ax2.annotate('', xy=(p_x, p2_z), xytext=(p_x, p1_z),
             arrowprops=dict(arrowstyle='->', color='black', lw=2))

# Add velocity vector labels
ax2.text(p_x + 0.15, (p1_z + p2_z)/2, r'$V_x = 1.0 \frac{m}{s}$', fontsize=12, fontweight='bold', color='#2c3e50')

ax2.set_xlabel('Lateral X (Meters)', fontsize=12)
ax2.set_ylabel('Forward Z (Meters)', fontsize=12)
ax2.legend(loc='upper right')

# ─── 5. RENDER AND SAVE ─────────────────────────────────────────────────────
plt.tight_layout()
plt.savefig('ipm_visualization.png', dpi=300, bbox_inches='tight')
print("Plot saved as 'ipm_visualization.png'")
# plt.show() # Disabled so it runs silently in bash
