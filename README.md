# DRIFT: Distributed Real-time Inertial and Frame Tracking on a $10 Bare-Metal MCU

<p align="center">
  <img src="https://img.shields.io/badge/ESP32--S3-Dual%20Core%20240%20MHz-red?logo=espressif&logoColor=white" />
  <img src="https://img.shields.io/badge/FreeRTOS-Bare%20Metal-darkgreen?logo=freertos&logoColor=white" />
  <img src="https://img.shields.io/badge/PlatformIO-Build-orange?logo=platformio&logoColor=white" />
  <img src="https://img.shields.io/badge/OV3660-Monocular-blue" />
  <img src="https://img.shields.io/badge/IMU-W%C3%BCrth%20ISDS%20%2F%20MPU--6050-purple" />
  <img src="https://img.shields.io/badge/EKF-6%20DOF%20Planar-blueviolet" />
  <img src="https://img.shields.io/badge/License-MIT-green" />
</p>

A complete **Visual-Inertial Odometry (VIO)** pipeline running entirely bare-metal on a single **Seeed Studio XIAO ESP32-S3 Sense** (240 MHz dual-core LX7, 8 MB PSRAM, $10 BOM). To survive the MCU compute budget we abandon 3D epipolar geometry: a **FAST-9 + Lucas-Kanade** front-end produces 2D pixel flow at 20 Hz, which is projected directly onto the ground plane via **Inverse Perspective Mapping (IPM)** for *O(N)* metric velocity extraction with no SVD, no Essential matrix, and no RANSAC. The residual monocular rotation-vs-translation ambiguity is resolved by hardware-stacking the camera and IMU and **pre-compensating pixel tracks with the IMU z-gyroscope** before metric extraction. The corrected planar velocity and integrated yaw drive a **6-state planar Extended Kalman Filter** that fuses 200 Hz inertial prediction with 20 Hz vision update under a FreeRTOS dual-core schedule. Four physical nodes vary IMU grade (industrial Würth WSEN-ISDS vs. hobby InvenSense MPU-6050) and fusion mode (VIO vs. inertial dead-reckoning) so the marginal value of vision and silicon are independently quantified.

<p align="center">
  <a href="80_Docu/FinalPaper/main.pdf">
    <img src="https://img.shields.io/badge/Paper-IEEE%20Conference-darkred?style=for-the-badge" />
  </a>
</p>

---

## System Architecture

<p align="center">
  <img src="80_Docu/FinalPaper/figures/architecture.png" width="100%" alt="DRIFT system architecture"/>
  <br/>
  <em><b>Figure 1.</b> Three-lane swim diagram of the firmware. Core 0 services the IMU at 200 Hz and pushes samples into a FreeRTOS queue. Core 1 consumes the queue, runs EKF prediction on every IMU sample, and at 20 Hz captures a frame and runs FAST-9 → Lucas-Kanade → IPM → gyro derotation → median + rotation gate → EKF measurement update. The dashed feedback arrow shows the Δψ pulled from the EKF yaw between camera frames. (Source: <code>80_Docu/FinalPaper/main.tex</code>, Fig. 1.)</em>
</p>

### Pipeline tasks (Node 1, Node 2)

| Task | Core | Priority | Period | Role |
|------|:---:|:---:|:---:|------|
| `imuTask` | 0 | 5 | 5 ms (200 Hz) | Polls IMU over I²C, applies static calibration, enqueues `ImuData` |
| `fusionTask` | 1 | 4 | queue-driven | EKF predict on every sample; 20 Hz cam capture → FAST → LK → IPM → derotate → median → EKF update |
| `telemetryTask` | 1 | 2 | 20 ms (50 Hz) | Streams state vector as JSON UDP packet (off-board logging) |

Nodes 3 and 4 (inertial dead-reckoning baselines) run only `imuTask` + a stripped fusion task that calls `ekfPredict` and skips the camera path.

---

## Tech stack

| Component | Choice |
|-----------|--------|
| MCU | Seeed Studio XIAO ESP32-S3 Sense — dual-core Tensilica LX7, 240 MHz, 512 kB SRAM, 8 MB PSRAM |
| Camera | OmniVision OV3660 (on-board), QVGA 320×240 grayscale at 20 Hz, SPI/DMA capture into PSRAM |
| IMU (Node 1, 3) | Würth WSEN-ISDS 6-axis MEMS, I²C |
| IMU (Node 2, 4) | InvenSense MPU-6050 6-axis MEMS, I²C |
| RTOS | FreeRTOS on Arduino-ESP32 framework, both cores pinned, no dynamic alloc after boot |
| Build | PlatformIO with per-node `platformio.ini` |
| Front-end | FAST-9 corner detection (Rosten et al.) + sparse Lucas-Kanade flow (Lucas-Kanade, Bouguet pyramidal) — both implemented from scratch in `10_src/node1_vio_WE_IMU/src/fast_corner.cpp` and `optical_flow.cpp` |
| Geometry | Inverse Perspective Mapping (Mallot et al.) on a flat ground plane — closed-form in `ipm.cpp` |
| Linear algebra | `BasicLinearAlgebra` compiled for the LX7 FPU; 3×3 matrix inverse in <200 µs |
| Transport | WiFi UDP, 50 Hz JSON state packets to a host laptop for evaluation |

---

## How it works

### Hardware platform

Each node uses a Seeed Studio XIAO ESP32-S3 Sense module: a 240 MHz dual-core Tensilica LX7 with 512 kB on-chip SRAM and 8 MB external PSRAM. The on-board OV3660 captures at QVGA (320×240) and is converted to grayscale before tracking (~75 kB working set per frame, resident in PSRAM). The IMU is mounted directly above the camera centre on a 3D-printed stack so that the camera principal point and the IMU package coincide laterally — this co-location eliminates a lever-arm term from the derotation step below.

### FAST-9 + Lucas-Kanade front-end

Each frame is reduced to up to 100 FAST-9 corners on the grayscale image. A high-speed reject test examining only the four cardinal pixels of the 16-pixel Bresenham circle eliminates the vast majority of non-corners before the full arc test fires. Corners are graded by a sum-of-absolute-differences score against the centre pixel and run through an 8-px non-maximum suppression.

Surviving corners are tracked into the next frame with a sparse Lucas-Kanade solver on a 7×7 window, central-difference gradients, bilinear sub-pixel sampling, and a maximum of five Gauss-Newton iterations per feature. Features whose 2×2 structure tensor has a determinant below `LK_MIN_DET = 1.0` are rejected as poorly conditioned. The output is a list of 2D flow vectors $\{(\mathbf{p}_i, \Delta\mathbf{p}_i)\}_{i=1}^{N}$ in pixel space.

### Inverse Perspective Mapping

Given the camera intrinsics $f_x, f_y, c_x, c_y$, the camera height $H$ above the floor, and the mounted pitch $\theta$ relative to horizon, every pixel $(u,v)$ maps deterministically to a ground point $(X,Y)$ under the flat-floor assumption. The mapping is closed-form:

```
α  = atan( (v − c_y) / f_y )            // elevation of pixel ray
Y  = H / tan(θ + α)                     // forward distance to ground hit
β  = atan( (u − c_x) / f_x )            // bearing
ρ  = √(H² + Y²)                         // ground-ray length
X  = ρ · tan(β)                         // lateral distance
```

Pixels with $\theta + \alpha \le 0.01$ rad point at or above the horizon and are rejected. Each call is *O(1)* using single-precision `tanf` / `atanf` intrinsics, with no matrix ops.

### Gyro-derotated optical flow — the key insight

A naive IPM applied to raw pixel flow conflates two distinct motions: a translational ground-plane velocity and a yaw rotation about the camera vertical axis. Both produce displacement on the ground plane and a monocular pipeline cannot disambiguate them by geometry alone. **This is what kills naive ground-plane VIO under turns.**

We resolve the ambiguity using the IMU. Let $\Delta\psi$ be the integrated yaw between two consecutive camera frames (pulled from the EKF state). For each tracked feature with old pixel $\mathbf{p}_i$ and new pixel $\mathbf{p}_i + \Delta\mathbf{p}_i$, both are projected to ground points $\mathbf{g}_i$ and $\mathbf{g}_i'$. The old ground point is then rotated by the IMU yaw delta:

```
g̃_i = R(Δψ) · g_i      with R(Δψ) the 2D rotation matrix
v_i = (g̃_i − g_i') / Δt
```

so any apparent ground motion now contains *only* translational components. We take the **per-axis median** across all valid features to get $(\hat{v}_x, \hat{v}_y)$ — robust to LK outliers, specular highlights, and moving objects — and report a confidence $c = N_{\text{valid}} / N_{\text{total}}$.

**Rotation gate.** During fast in-plane rotation, even small errors in Δψ leak as spurious translational velocity. We drop the camera update entirely whenever $|\Delta\psi / \Delta t| > 0.5$ rad/s and trust IMU prediction alone for those frames. The exact check is in `fusionTask` (`main.cpp`, ~line 187).

### Six-state planar EKF

State vector:

```
x = [ p_x, p_y, v_x, v_y, ψ, b_ω_z ]ᵀ ∈ ℝ⁶
```

`p` is 2D position in the world frame, `v` is planar velocity, `ψ` is yaw, and `b_ω_z` is the gyro z-bias estimate.

**Predict** (every 5 ms IMU sample): rotate body-frame acceleration into world frame by current yaw, integrate to update position and velocity; integrate bias-corrected angular rate `ω̃_z = ω_z − b_ω_z` to update yaw. The 6×6 Jacobian `F` is constructed analytically; covariance propagates as `P ← F·P·Fᵀ + Q`.

**Update** (every 50 ms valid camera frame): with linear `H` selecting rows `[v_x, v_y, ψ]` and confidence-modulated noise `R = (R_0 / (c + ε)) · I_3`, apply the standard EKF correction. The yaw innovation is wrapped to `[−π, π)` before correction. Matrix inversion uses `BasicLinearAlgebra` on the LX7 FPU; 3×3 takes <200 µs.

### Implementation parameters

The full parameter table — camera, IPM, FAST, LK, EKF, FreeRTOS — is in the paper (Table II of `80_Docu/FinalPaper/main.pdf`). Key values:

| Group | Parameter | Value |
|-------|-----------|------:|
| Camera | Resolution | 320×240 grayscale |
| Camera | Frame rate | 20 Hz |
| Camera / IPM | Mount height `H` | 0.05 m |
| Camera / IPM | Pitch `θ` | 15° downward |
| IPM | Focal length `f_x, f_y` | 500 px |
| FAST-9 | Intensity threshold | 20 |
| FAST-9 | NMS radius | 8 px |
| LK | Window size | 7×7 px |
| LK | Max iterations | 5 |
| Vel. extraction | Min valid features | 3 |
| Vel. extraction | Rotation gate | 0.5 rad/s |
| EKF | `q_v_x, q_v_y` | 1×10⁻² |
| EKF | `q_ψ` | 5×10⁻⁴ |
| EKF | `q_b_ω_z` | 1×10⁻⁵ |
| EKF | Camera meas. base noise `R_0` | 1×10⁻³ |
| RTOS | IMU rate | 200 Hz |
| RTOS | IMU queue depth | 20 samples |

---

## Build

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
- USB-C cable for the XIAO ESP32-S3
- Python 3.9+ (for host-side tools in `60_scripts/`)

### Per-node build

Each node is a self-contained PlatformIO project under `10_src/`. Open the folder in VS Code and PlatformIO resolves dependencies from the per-node `platformio.ini`.

```bash
# Node 1 — Würth ISDS + VIO (the canonical pipeline)
cd 10_src/node1_vio_WE_IMU
pio run                   # compile
pio run --target upload   # compile + flash over USB
pio device monitor        # 115200 baud

# Node 2 — MPU-6050 + VIO
cd 10_src/node2_vio_MPU_IMU
pio run --target upload

# Node 3 — Würth ISDS only (inertial baseline)
cd 10_src/node3_WE_imu
pio run --target upload

# Node 4 — MPU-6050 only (inertial baseline)
cd 10_src/node4_MPU_imu
pio run --target upload
```

> **Upload issue:** PlatformIO's default upload speed (921600 baud) can cause a serial-handshake failure on the XIAO. Falling back to `upload_speed = 460800` in `platformio.ini` (or using `esptool.py` directly) works around it.

---

## Run

### Compile-time flags (Node 1, Node 2)

Edit the top of `10_src/node1_vio_WE_IMU/src/main.cpp` (or the Node 2 equivalent):

```cpp
constexpr bool VISION_ENABLED = true;   // false → IMU-only EKF, no camera path
constexpr bool WIFI_ENABLED   = false;  // true  → broadcast state over UDP
constexpr bool DEBUG_PRINT    = true;   // false → suppress serial output
```

`VISION_ENABLED = false` collapses Node 1 → Node 3 (or Node 2 → Node 4) for direct on-the-same-hardware comparison.

### Serial output

After flashing Node 1, the boot sequence looks like:

```
[NODE 1] PROJECT DRIFT — VIO Node booting...
Calibrating — keep sensor still...
=== Calibration Complete ===
Offsets ax:0.0023 ay:-0.0011 az:0.0004
Gyro offsets gx:12.34 gy:-8.21 gz:47.83 mdps
[NODE 1] Camera initialized successfully.
[NODE 1] EKF Initialized.
[NODE 1] Setup complete.
[EKF] X:   0.000 m  Y:   0.000 m  Yaw:   0.000 deg  bgz:+0.00 mdps  conf:0.00
[EKF] X:   0.178 m  Y:   0.115 m  Yaw:  -3.991 deg  bgz:+120.3 mdps  conf:0.94
```

The `conf` field is the fraction of LK-tracked features that survived the IPM + median filter; values above ~0.5 indicate a healthy ground texture. Drops to 0 indicate the camera lost track (textureless floor, motion blur, or all features rejected by the rotation gate).

### Off-board telemetry

With `WIFI_ENABLED = true`, the node broadcasts a JSON state packet at 50 Hz over UDP. The host-side receiver lives in `60_scripts/` — see `60_scripts/README.md`.

---

## Repository layout

```
esp32-vio-drift/
├── README.md                       ← This file
├── 10_src/                         ← Firmware (PlatformIO projects, one per node)
│   ├── node1_vio_WE_IMU/           ← N1: WE ISDS + VIO (canonical pipeline)
│   │   ├── platformio.ini
│   │   ├── include/                ← ipm.h, ekf.h, fast_corner.h, optical_flow.h …
│   │   └── src/                    ← main.cpp, vio_processor.cpp, ekf.cpp, ipm.cpp …
│   ├── node2_vio_MPU_IMU/          ← N2: MPU-6050 + VIO
│   ├── node3_WE_imu/               ← N3: WE ISDS only, dead-reckoning baseline
│   └── node4_MPU_imu/              ← N4: MPU-6050 only, dead-reckoning baseline
├── 20_docs/                        ← Design documentation
│   ├── architecture/               ← System design notes, pipeline diagrams
│   ├── wiring/                     ← Pin maps, I²C wiring guides
│   ├── calibration/                ← IMU & camera calibration procedures
│   └── api/                        ← Per-module API documentation
├── 30_data/                        ← Sensor logs & evaluation runs
├── 40_models/                      ← TinyML models (Edge Impulse → TFLite Micro, stretch)
├── 50_evaluation/                  ← Results, drift analysis, plots
├── 60_scripts/                     ← Host-side Python tooling (UDP receiver, plotters)
├── 70_hardware/                    ← BOM, datasheets, hardware reference
└── 80_Docu/
    └── FinalPaper/                 ← IEEE conference paper
        ├── main.tex
        ├── main.pdf
        └── references.bib
```

---

## Key design decisions

| Decision | Rationale |
|----------|-----------|
| **Skip 3D reconstruction; use IPM** | Multi-view geometry (SVD on the Essential matrix, RANSAC inliers, triangulation) is the dominant compute cost in conventional VIO. With a known camera height and pitch, every pixel maps deterministically to a metric ground point — pixel velocity becomes metric velocity at *O(N)* cost. |
| **Hardware-stack the IMU directly over the camera centre** | Eliminates the lever-arm term from gyro derotation; Δψ from the IMU is the same yaw the camera sees. |
| **Gyro-derotate before IPM, not after** | Rotating in pixel space is awkward; rotating the projected ground point is a 2×2 matrix multiply with `cos(Δψ)`, `sin(Δψ)` precomputed once per frame. |
| **Per-axis median for velocity, not mean** | Robust to LK divergence, specular highlights on the floor, and small moving objects in the field of view. Mean would be hostage to a single bad track. |
| **Rotation gate at 0.5 rad/s** | Imperfect derotation under fast spin leaks as a phantom translation through the yaw↔v_x cross-covariance; better to drop the camera update than to corrupt the filter. |
| **Confidence-modulated camera R** | `R = R_0 / (c + ε)` — high-feature-count, ground-rich frames get a tight measurement noise; texture-collapse frames are softly down-weighted instead of hard-rejected. |
| **6-state planar EKF, not full 15-state SE(3)** | A flat-floor ground robot doesn't observe the out-of-plane states reliably, so keeping them in the filter only adds noise and compute cost. |
| **FreeRTOS Core 0 for IMU, Core 1 for fusion** | Guarantees that SPI/DMA camera traffic and EKF compute on Core 1 cannot stall the deterministic 200 Hz inertial path on Core 0. |
| **No dynamic memory after `setup()`** | All per-frame buffers are stack-allocated or caller-provided. Removes heap fragmentation as a failure mode in long runs. |

---

## Results

Full evaluation — methodology, scenarios, and per-cell endpoint drift across four nodes and five trajectories — is in the [paper](80_Docu/FinalPaper/main.pdf), §V. Headline numbers:

| | Static (10 s) | Straight (2 m) | Rectangle (1×0.5 m) | Circle (⌀ 40 cm) | Free-form |
|---|---:|---:|---:|---:|---:|
| **N1: WE + VIO**    | 0.030 m | 0.028 m | 0.065 m | 0.110 m | 0.130 m |
| **N2: MPU + VIO**   | 0.043 m | 0.030 m | 0.085 m | 0.140 m | 0.180 m |
| **N3: WE only**     |  6.50 m |  1.05 m |  2.40 m |  4.20 m |  6.80 m |
| **N4: MPU only**    |  13.0 m |  2.10 m |  4.80 m |  8.50 m |  13.5 m |

The headline: **visual fusion compresses the IMU-grade gap.** In dead-reckoning mode the industrial Würth and hobby MPU sit ~2× apart on every scenario, with both diverging to multi-metre drift even on the static trajectory (gravity coupling through tilt-estimate error dominates). Under VIO, both IMU configurations collapse to the same centimetre band — a hobby IMU with vision is competitive with an industrial IMU operating alone. The largest single-cell ratio is **N4 vs. N2 on static: 13.0 m → 0.043 m, a 302× drift reduction.**

Caveat: the per-cell entries above are theoretical projections derived from manufacturer sensor noise specifications and the IPM-VIO error model. Empirical campaign is in progress and will replace projected entries as data becomes available. See the table caption in `main.tex` for the live status.

---

## Future work

- **TinyML motion-class auxiliary channel** — a quantised classifier (Edge Impulse → TFLite Micro) consuming a one-second window of `(a_x, a_y, ω_z)` plus EKF velocity, emitting `{stationary, straight, turning, erratic}`. Lets the EKF tighten Q in stationary regimes and widen R during erratic motion. Targets <5 ms inference on Core 1.
- **Camera intrinsic recalibration at QVGA** — the current IPM constants (`f_x = f_y = 500`, `c_x = 320`, `c_y = 240`) were calibrated against a 640×480 image plane; the QVGA capture path now produces a 320×240 frame, so the focal length and image centre should be re-derived to match. Open work item.
- **Ground-plane assumption relaxation** — add a slope-estimation feedback from the IPM residual to handle non-flat floors and small ramps without abandoning the closed-form mapping.
- **Bias observability via structured trajectories** — explore whether figure-8s, S-turns, and other excitation patterns keep `b_ω_z` observable from the camera channel alone over long runs, removing the need for stationary-phase auxiliaries.
- **Full empirical campaign** — execute 10 runs per (node, scenario) cell across the four nodes and replace the projected entries in the results table.

---

## Citation

If you use this work, please cite the paper:

```bibtex
@inproceedings{Gery2026Drift,
  title  = {DRIFT: Distributed Real-time Inertial and Frame Tracking
            on a \$10 Bare-Metal Microcontroller},
  author = {Gery, Phillipp and Benitez, Francisco and Pisciotta, Sam
            and Patkar, Vedant and Kester, Jack},
  year   = {2026}
}
```

---

## Authors

**Phillipp Gery** (Team Lead) — system architecture, EKF design, gyro-derotation • `pgery@purdue.edu`
**Francisco Benitez** — XIAO ESP32-S3 Sense / OV3660 driver, FAST corners, calibration
**Sam Pisciotta** — RTOS firmware, bare-metal drivers, data pipeline across all four nodes
**Vedant Patkar** — sensor fusion, optical-flow tuning, TinyML stretch
**Jack Kester** — evaluation harness, drift analysis, visualisation

School of Electrical and Computer Engineering, Purdue University.

The authors thank **Würth Elektronik** for providing the WSEN-ISDS IMU evaluation samples used in Nodes 1 and 3.
<p align="center">
  <img src="80_Docu/FinalPaper/figures/Wuerth_Elektronik-Gruppe.jpg" width="50%" alt="DRIFT system architecture"/>
  <br/>
</p>

## License

MIT
