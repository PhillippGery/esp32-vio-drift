
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ─── 1. PHYSICAL PARAMETERS ────────────────────────────────────────────────
h = 0.4          # Camera height in meters (e.g., 50cm off ground)
pitch_deg = 25.0 # Pitch angle (degrees down from horizontal)
fov_deg = 40.0   # Vertical Field of view

pitch = np.radians(pitch_deg)
fov_half = np.radians(fov_deg / 2.0)

# ─── 2. CALCULATE RAY ANGLES (Positive is Downward) ────────────────────────
angle_opt = pitch
angle_lower = pitch + fov_half  # 20 + 30 = 50 degrees down
angle_upper = pitch - fov_half  # 20 - 30 = -10 degrees (POINTING UP!)

# ─── 3. INTERSECTIONS ──────────────────────────────────────────────────────
x_opt = h / np.tan(angle_opt)
x_lower = h / np.tan(angle_lower)

# ─── 4. PLOT SETUP ─────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 5))
fig.suptitle('IPM Geometry Trap: 20° Pitch vs 60° FOV', fontsize=16, fontweight='bold', color='#c0392b')

# Draw ground
ax.axhline(0, color='black', linewidth=2)

# Draw rover body
rover = patches.Rectangle((-0.3, 0), 0.3, h, linewidth=1, edgecolor='black', facecolor='#7f8c8d')
ax.add_patch(rover)
ax.plot(0, h, 'ko', markersize=8, label='Camera Node 1')

# ─── 5. DRAW RAYS ──────────────────────────────────────────────────────────
# Lower FOV Ray (Hits ground safely)
ax.plot([0, x_lower], [h, 0], color='#27ae60', linewidth=2.5, label='Lower FOV (Floor Intersection)')

# Optical Axis (Hits ground)
ax.plot([0, x_opt], [h, 0], color='#f39c12', linestyle='--', linewidth=2, label='Optical Axis (Center of Frame)')

# Upper FOV Ray (Shoots into the sky)
# Calculate a point far out just to draw the line
x_upper_plot = 5.0
y_upper_plot = h - (x_upper_plot * np.tan(angle_upper)) 
ax.plot([0, x_upper_plot], [h, y_upper_plot], color='#c0392b', linewidth=2.5, label='Upper FOV (Sky / Walls - MATHEMATICAL TRAP)')

# Highlight valid ground area
ax.axvspan(x_lower, 5.0, color='#2ecc71', alpha=0.15)
ax.text((x_lower + 5)/2, 0.05, 'Valid IPM Projection Area', color='#27ae60', fontweight='bold', ha='center')

# ─── 6. FORMATTING ─────────────────────────────────────────────────────────
ax.set_xlim(-0.5, 5)
ax.set_ylim(-0.2, h + 0.5)
ax.set_xlabel('Forward Distance Z (meters)', fontsize=12, fontweight='bold')
ax.set_ylabel('Height Y (meters)', fontsize=12, fontweight='bold')
ax.grid(True, linestyle=':', alpha=0.7)
ax.legend(loc='upper right')

plt.tight_layout()
plt.savefig('ipm_20deg_geometry.png', dpi=300, bbox_inches='tight')
print("Plot saved as 'ipm_20deg_geometry.png'")
plt.show()
