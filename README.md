# PROJECT DRIFT — Visual-Inertial Odometry on a 5-Node ESP32 Network

> **Repository:** `esp32-vio-drift`

---

## Overview

PROJECT DRIFT investigates real-time **Visual-Inertial Odometry (VIO)** on
resource-constrained embedded hardware. A primary VIO node fuses IMU readings
(Würth ISDS 2536030320001) with downward-facing camera frames (XIAO ESP32S3 Sense)
using a 6-state Extended Kalman Filter running on an ESP32-S3. Four additional
IMU-only nodes broadcast raw sensor data over WiFi UDP as baseline comparison
measurements for evaluating the VIO pipeline's drift suppression.

---

## Team Roles

| Name | Role |
|---|---|
| **Phillipp Gery** | Team Lead · Kalman Filter design & implementation · Project Structure |
| **Panchtio** | Camera integration (XIAO ESP32S3 Sense, optical flow pipeline) |
| **Sam** | Firmware architecture, PlatformIO build system, Node 2–5 firmware |
| **Vedant** | Sensor fusion, TinyML pipeline (Edge Impulse → TFLite Micro) |
| **Jack** | Evaluation, drift analysis, visualization dashboard |

---

## Hardware

### Node 1 — VIO Node (primary)

| Component | Role |
|---|---|
| XIAO ESP32S3 | Dual-core microcontroller (240 MHz) |
| Würth ISDS 2536030320001 | 6-axis IMU via I²C — ±250 dps / ±4g, 208 Hz |
| XIAO ESP32S3 Sense (OV3660) | Downward-facing camera for optical flow |

### Nodes 2–5 — IMU Reference Nodes (comparison only)

| Component | Role | Qty |
|---|---|---|
| ESP32 Feather V2 (Adafruit) | Microcontroller | 4 |
| MPU-6050 | 6-axis IMU via I²C — ±250 dps / ±2g, 200 Hz | 4 |

### Node 1.5 — Würth ISDS Dead Reckoner (comparison)

| Component | Role |
|---|---|
| ESP32 Feather V2 | Microcontroller |
| Würth ISDS 2536030320001 | Same sensor as Node 1, no camera, dead reckoning only |

> Nodes 2–5 and Node 1.5 exist solely to compare raw IMU dead-reckoning drift
> against the VIO pipeline on Node 1. They do not run an EKF or camera.

---

## Repository Structure

```
esp32-vio-drift/
├── 10_src/
│   ├── node1_vio/        Node 1: XIAO ESP32S3 + Würth ISDS + OV3660 (VIO + EKF)
│   ├── node1.5_imu/      Node 1.5: ESP32 + Würth ISDS (dead reckoning, comparison)
│   ├── node2_imu/        Node 2: ESP32 + MPU-6050 (dead reckoning, comparison)
│   └── node3_5_imu/      Nodes 3–5: shared MPU-6050 firmware (telemetry only)
├── 20_docs/              Design documentation
│   ├── architecture/     System design, EKF math, pipeline diagrams
│   ├── wiring/           Pin maps, I²C wiring guides
│   ├── calibration/      IMU & camera calibration procedures
│   └── api/              Function & module documentation
├── 30_data/              Sensor logs & ML datasets
├── 40_models/            TinyML models (Edge Impulse → TFLite Micro)
├── 50_evaluation/        Results & drift analysis
├── 60_scripts/           Host-side Python tooling
└── 70_hardware/          Hardware reference, BOM, datasheets
```

---

## Node 1 Firmware Architecture

Node 1 runs a **FreeRTOS dual-core pipeline** on the XIAO ESP32S3:

```
Core 0 — imuTask (Priority 5, 208 Hz)
  └─ Polls Würth ISDS via I²C
  └─ Computes actual elapsed dt (micros-based)
  └─ Pushes ImuData → FreeRTOS queue (depth 20)

Core 1 — fusionTask (Priority 4, queue-driven)
  └─ ekfPredict(ax, ay, gz, dt)          ← IMU dead-reckoning step
  └─ ekfZupt(gz) when stationary         ← bias correction (see below)
  └─ cameraProcessFrame() at 20 Hz       ← optical flow → metric velocity
  └─ ekfUpdateCamera(vx, vy, yaw, conf)  ← EKF measurement update

Core 1 — telemetryTask (Priority 2, 50 Hz)
  └─ sendOdometryPacket() over WiFi UDP  ← optional, WIFI_ENABLED flag
```

### EKF State Vector

```
x = [px, py, vx, vy, yaw, bgz]^T
```

| State | Description |
|---|---|
| `px`, `py` | 2D position (m) |
| `vx`, `vy` | Velocity in global frame (m/s) |
| `yaw` | Heading (rad) |
| `bgz` | Gyro Z bias estimate (rad/s) — corrected online by ZUPT |

---

## Drift Compensation Pipeline

### Problem

MEMS gyroscopes have a temperature-dependent bias (ZRO drift). After boot
calibration, the bias shifts 5–10 mdps/°C as the chip warms up. Integrating
even a small residual bias continuously generates yaw drift.

### Multi-layer solution implemented on Node 1

**Layer 1 — Sensor configuration**
- Full scale: ±250 dps (not the default 500/2000 dps) → 8.75 mdps/LSB quantization
- Hardware LPF1 enabled at 208 Hz ODR
- Software `MovingAvg<5>` on gz output (matches MPU-6050 driver filtering)

**Layer 2 — Extended calibration**
- 1 s hard delay + 100 throwaway samples before collecting bias estimates
- 500 averaging samples (2.5 s) — sensor thermally settled before measurement

**Layer 3 — ZUPT (Zero-velocity / Zero-rate Update)**

The core innovation. The EKF state has a `bgz` field, but the original
`ekfUpdateCamera` H matrix had a zero in the bgz column — the bias state
was never observable from camera measurements. ZUPT fixes this directly.

When the robot is confirmed stationary (dual-sensor gate):
```
acc_h = sqrt(ax² + ay²) < 0.3 m/s²    (no horizontal acceleration)
|gz| < 0.008 rad/s                      (no rotation)
```

`ekfZupt(gz)` is called, injecting a measurement with H observing `[vx, vy, bgz]`:
```
z = [0, 0, gz]    (when still: true rate = 0, so gz = thermal bias)
R_bgz = 0.02      (smooth convergence, not per-sample tracking)
```

**Result (measured):**

| Condition | Yaw drift |
|---|---|
| Before ZUPT, 2000 dps full scale | Rapid (degrees/min) |
| After full-scale + filter fixes only | Similar — dominant source was thermal bias, not noise |
| After ZUPT (current) | **~0.016 deg/min at rest** |

The bgz estimate converges to the actual thermal offset (~120 mdps in typical
testing) within ~1 second of stationary time, then freezes that correction
for use during motion.

### VIO correctness during in-place rotation

When rotating in place, the optical flow processor (`vio_processor.cpp`) uses
`delta_yaw_imu` (from the EKF) to de-rotate each ground-projected feature point
before computing metric velocity:

```
v = (R(delta_yaw) * p_old - p_new) / dt
```

For pure rotation: `R(delta_yaw) * p_old ≈ p_new` → `v ≈ 0`. The EKF receives
`meas_vx ≈ 0, meas_vy ≈ 0`, constraining velocity to zero while yaw integrates
freely from `gz - bgz`. Position does not drift during in-place rotation.
ZUPT does not fire during rotation (`|gz| > 0.008`).

---

## Runtime Flags (Node 1)

Edit at the top of `10_src/node1_vio/src/main.cpp`:

```cpp
constexpr bool VISION_ENABLED = true;   // false → IMU-only EKF (no camera)
constexpr bool WIFI_ENABLED   = false;  // true  → enable UDP telemetry
constexpr bool DEBUG_PRINT    = true;   // false → suppress serial output
```

---

## PlatformIO Setup

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with the
  [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
- Python 3.9+ (for `60_scripts/`)

### Opening a Node Project

1. Open VS Code.
2. In PlatformIO Home, click **Open Project**.
3. Navigate to the desired node folder, e.g. `10_src/node1_vio/`.
4. PlatformIO resolves all dependencies defined in `platformio.ini`.

### Building & Flashing

```bash
# From inside a node project folder:
pio run                   # compile
pio run --target upload   # compile + flash over USB
pio device monitor        # open serial monitor (115200 baud)
```

### Serial Output (Node 1)

```
Calibrating — keep sensor still...
=== Calibration Complete ===
Offsets ax:0.0023 ay:-0.0011 az:0.0004
Gyro offsets gx:12.34 gy:-8.21 gz:47.83 mdps   ← verify gz is small

[EKF] X:   0.000 m  Y:   0.000 m  Yaw:   0.000 deg  bgz:+0.00 mdps  conf:0.00
...
[EKF] X:   0.178 m  Y:   0.115 m  Yaw:  -3.991 deg  bgz:+120.3 mdps  conf:0.94
```

`bgz` should drift from 0 toward a stable value (typically ±50–200 mdps) within
the first second of stationary time. If it stays at 0, the ZUPT detection
threshold may need tuning for your environment.

---

## Python Environment

```bash
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r 60_scripts/requirements.txt
```

---

## Branch Strategy

| Branch | Purpose |
|---|---|
| `main` | Stable, tagged releases only |
| `dev` | Integration branch — all PRs merge here first |
| `feature/*` | Individual feature branches |

---

## License

All rights reserved by the project team.
