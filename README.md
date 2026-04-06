# PROJECT DRIFT — Visual-Inertial Odometry on a 5-Node ESP32 Network

> **Repository:** `esp32-vio-drift`

---

## Overview

PROJECT DRIFT investigates real-time **Visual-Inertial Odometry (VIO)** on
resource-constrained embedded hardware. A primary VIO node fuses IMU readings
(MPU-6050) with camera frames (XIAO ESP32S3 Sense) using an Extended Kalman
Filter running directly on an ESP32 Feather V2. Four additional IMU-only nodes
broadcast raw sensor data over the same network for reference measurements and
stress-testing the fusion pipeline.

A **TinyML** stretch goal (Week 5) introduces an Edge Impulse–trained motion
classifier compiled to TFLite Micro and deployed on the VIO node.

---

## Team Roles

| Name | Role |
|---|---|
| **Phillipp Gery** | Team Lead · Kalman Filter design & implementation & Project Structure |
| **Panchtio** | Camera integration (XIAO ESP32S3 Sense, frame capture pipeline) |
| **Sam** | Firmware architecture, PlatformIO build system, Node 2–5 firmware |
| **Vedant** | Sensor fusion, TinyML pipeline (Edge Impulse → TFLite Micro) |
| **Jack** | Evaluation, drift analysis, visualization dashboard |

---

## Hardware

| Component | Role | Qty |
|---|---|---|
| ESP32 Feather V2 (Adafruit) | Microcontroller for all 5 nodes | 5 |
| MPU-6050 | 6-axis IMU (accel + gyro) via I²C | 5 |
| XIAO ESP32S3 Sense (OV2640) | 2 MP SPI camera — Node 1 only | 1 |
| Micro-USB cables | Programming & power | 5 |
| Breadboards + jumper wires | Prototyping | — |

---

## Repository Structure

```
esp32-vio-drift/
├── 10_src/               ← PlatformIO firmware projects
│   ├── node1_vio/        Node 1: ESP32 + MPU-6050 + ESP32S3 Sense (VIO)
│   ├── node2_imu/        Node 2: ESP32 + MPU-6050 (IMU reference)
│   └── node3_5_imu/      Nodes 3–5: shared firmware (identical to Node 2)
├── 20_docs/              ← Design documentation
│   ├── architecture/     System design, EKF math, pipeline diagrams
│   ├── wiring/           Pin maps, I²C wiring guides
│   ├── calibration/      IMU & camera calibration procedures
│   └── api/              Function & module documentation
├── 30_data/              ← Sensor logs & ML datasets
│   ├── raw_imu/          Raw CSV/binary logs from all 5 nodes
│   ├── labeled_datasets/ Manually labeled motion segments
│   └── edgeimpulse_export/ Edge Impulse dataset exports
├── 40_models/            ← TinyML models
│   ├── tflite_micro/     Quantized INT8 .tflite models + C arrays
│   └── training_logs/    Edge Impulse training logs & model cards
├── 50_evaluation/        ← Results & analysis
│   ├── test_results/     Drift comparison tables, raw test data
│   └── plots/            Drift plots, classifier accuracy charts
├── 60_scripts/           ← Host-side Python tooling
│   ├── data_pipeline/    Serial/WiFi data capture scripts
│   ├── visualization/    Jack's dashboard & plotting scripts
│   └── utils/            Calibration tools, log parsers
└── 70_hardware/          ← Hardware reference
    ├── bom/              Bill of Materials
    └── datasheets/       Component datasheets
```

---

## PlatformIO Setup

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with the
  [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
- Python 3.9+ (for `60_scripts/`)

### Opening a Node Project

1. Open VS Code.
2. In PlatformIO Home, click **"Open Project"**.
3. Navigate to the desired node folder, e.g. `10_src/node1_vio/`.
4. PlatformIO will automatically resolve all dependencies defined in
   `platformio.ini`.

### Building & Flashing

```bash
# From inside a node project folder:
pio run                   # compile
pio run --target upload   # compile + flash over USB
pio device monitor        # open serial monitor (115200 baud)
```

### Serial Monitor

All nodes transmit JSON-formatted sensor packets at **115200 baud**.
Node 1 also streams camera metadata.

---

## Python Environment (60_scripts/)

```bash
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r 60_scripts/requirements.txt
```

---

## Branch Strategy

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for the full branching model.

| Branch | Purpose |
|---|---|
| `main` | Stable, tagged releases only |
| `dev` | Integration branch — all PRs merge here first |
| `feature/*` | Individual feature branches |

---

## License

All rights reserved by the project team.
