# Visualization Scripts

**Owner:** Jack

| Script | Description |
|---|---|
| `dashboard.py` | Real-time Dash/Plotly web dashboard (live IMU + EKF state) |
| `plot_drift.py` | Post-run drift trajectory plots (2D/3D position over time) |
| `plot_accuracy.py` | TinyML classifier confusion matrix & accuracy bar charts |

## Running the Dashboard

```bash
python dashboard.py
# Open http://localhost:8050 in your browser
```

The dashboard connects to the UDP capture process and renders:
- Live 3D position estimate from Node 1 EKF
- Per-node IMU magnitude plots
- Gyro bias drift over time
