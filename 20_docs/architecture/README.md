# Architecture Documentation

Place system design artifacts here:

- `system_overview.md` — high-level VIO pipeline description
- `ekf_math.md` — Extended Kalman Filter derivation (state, predict, update equations)
- `network_topology.md` — 5-node WiFi UDP topology diagram
- `pipeline_diagram.png` — block diagram (draw.io or similar)

## VIO Pipeline Summary

```
[Node 1]
  MPU-6050 (200 Hz) ──► EKF Predict
  ArduCAM (5 Hz)    ──► Feature Extraction ──► EKF Update
                                                    │
                                              Position Estimate
                                                    │
                                            UDP JSON @ 50 Hz ──► Host PC

[Nodes 2–5]
  MPU-6050 (200 Hz) ──► JSON packet ──► UDP @ 100 Hz ──► Host PC
```
