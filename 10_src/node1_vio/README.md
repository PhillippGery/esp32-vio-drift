# PROJECT DRIFT — Node 1: Visual-Inertial Odometry Pipeline

Camera capture, FAST-9 corner detection, Lucas-Kanade sparse optical flow, and **multi-frame flow accumulation** running on the **Seeed XIAO ESP32S3 Sense** (OV3660) at 320x240 grayscale.

This is the visual frontend of the DRIFT VIO system. It produces filtered 2D displacement estimates from tracked image features, which feed directly into the Extended Kalman Filter for pose estimation.

## Pipeline

```
Camera Frame (320x240 grayscale, PSRAM double-buffered)
       |
       v
FAST-9 Corner Detection (20-50 corners, configurable threshold)
       |
       v
Lucas-Kanade Optical Flow (7x7 window, 5 iterations, sub-pixel)
       |
       v
Flow Accumulator (average over N=5 frames, variance + confidence)
       |
       v
VIO Measurement (dx_m, dy_m, confidence) --> EKF Update
```

## EKF Integration

Phillipp's EKF uses a 12-DOF state vector:

```
State = [px, py, pz, vx, vy, vz, roll, pitch, yaw, bx, by, bz]
          |_________|                                |__________|
          position (m)                               gyro bias (rad/s)
                      |_________|
                      velocity (m/s)
                                   |________________|
                                   orientation (rad)
```

The camera pipeline updates **px, py** (position) through the `VioMeasurement` struct:

```cpp
struct VioMeasurement {
    float delta_x_m;    // X displacement in meters
    float delta_y_m;    // Y displacement in meters
    float confidence;   // 0.0-1.0 (scales EKF measurement noise R)
    int64_t timestamp;  // Microsecond timestamp
    bool valid;
};
```

**Confidence maps to the EKF R matrix:** `R = base_R / confidence`. Higher confidence (more tracked features, low variance) means the filter trusts the camera more relative to IMU.

### Why Multi-Frame Accumulation?

Single-frame optical flow is noisy. By averaging flow vectors over 5 frames:

- **Noise reduction:** Random per-frame errors cancel out (sqrt(N) improvement)
- **Outlier suppression:** Variance tracking identifies unreliable measurements
- **IMU synergy:** The EKF can weight camera vs IMU based on the confidence score. When the camera sees good texture (high confidence), it dominates. On blank walls (low confidence), the IMU takes over.

### Pixel-to-Meter Conversion

Uses the pinhole camera model:

```
displacement_meters = (pixel_displacement * scene_depth) / focal_length_px
```

Current defaults (need calibration):
- `FOCAL_LENGTH_PX = 240.0` (estimated for OV3660 at QVGA)
- `BASELINE_DEPTH_M = 0.5` (assumed scene depth)

## Hardware

| Component | Spec |
|-----------|------|
| Board | Seeed XIAO ESP32S3 Sense |
| Camera | OV3660 (onboard FPC connector) |
| Resolution | 320x240 grayscale (QVGA) |
| PSRAM | 8 MB octal |
| CPU | ESP32-S3, 240 MHz dual-core |

## Project Structure

```
node1_vio/
|-- include/
|   |-- camera_pins.h         GPIO pin map (fixed by Sense board)
|   |-- fast_corner.h         FAST-9 detector interface
|   |-- optical_flow.h        LK optical flow interface
|   |-- flow_accumulator.h    Multi-frame flow averaging + VIO output
|-- src/
|   |-- main.cpp              Main pipeline loop
|   |-- fast_corner.cpp       FAST-9 with NMS
|   |-- optical_flow.cpp      LK optical flow
|-- tools/
|   |-- save_frame.py         Capture single frame -> PNG
|   |-- view_corners.py       Visualize FAST corners
|   |-- view_flow.py          Visualize corners + flow vectors
|   |-- live_view.py          Real-time OpenCV viewer
|   |-- live_stream.cpp       WiFi MJPEG stream (debug)
|-- platformio.ini
```

## Build and Flash

```bash
pio run                      # compile
pio run --target upload      # compile + flash
pio device monitor           # serial monitor (115200 baud)
```

## Serial Output

```
I (DRIFT) Pipeline: CAPTURE -> FAST -> LK -> ACCUM(5) -> EKF
I (DRIFT) cap:12.1 FAST:6.3 LK:18.4 ms | corners:28 flow:19 accum:3/5
I (DRIFT) cap:11.8 FAST:5.9 LK:17.2 ms | corners:31 flow:22 accum:4/5
I (DRIFT) cap:12.0 FAST:6.1 LK:18.0 ms | corners:29 flow:20 accum:5/5
I (DRIFT) VIO: dx=0.0042m dy=-0.0018m conf=0.76 (5 frames)
```

The `accum:3/5` counter shows frames accumulated toward the next VIO measurement. When it hits 5/5, a `VIO:` line prints the filtered displacement in meters with confidence.

## Python Debug Tools

```bash
pip install pyserial Pillow numpy opencv-python
```

**Close PlatformIO serial monitor before running any tool.**

| Tool | Command | Description |
|------|---------|-------------|
| save_frame.py | `python tools/save_frame.py --port COM5` | Single frame capture |
| view_corners.py | `python tools/view_corners.py --port COM5` | Frame + FAST overlay |
| view_flow.py | `python tools/view_flow.py --port COM5` | Frame + corners + flow arrows |
| live_view.py | `python tools/live_view.py --port COM5` | Real-time OpenCV window |

## Tuning Parameters

### main.cpp

| Parameter | Default | Effect |
|-----------|---------|--------|
| `FAST_THRESHOLD` | 20 | Corner intensity threshold |
| `NMS_RADIUS` | 8 | Min spacing between corners |
| `FOCAL_LENGTH_PX` | 240.0 | Camera focal length (calibrate!) |
| `BASELINE_DEPTH_M` | 0.5 | Assumed scene depth |

### flow_accumulator.h

| Parameter | Default | Effect |
|-----------|---------|--------|
| `ACCUM_WINDOW` | 5 | Frames to average (5-10) |

### optical_flow.h

| Parameter | Default | Effect |
|-----------|---------|--------|
| `LK_WINDOW` | 7 | Tracking patch size |
| `LK_MAX_ITER` | 5 | Max iterations per feature |
| `LK_EPSILON` | 0.01 | Convergence threshold |
| `LK_MIN_DET` | 1.0 | Rejects flat/textureless patches |

## Data Flow to Other Nodes

```
Node 1 (this) ----serial----> Host PC -----> Evaluation Dashboard (Jack)
     |
     +---> VioMeasurement --> EKF (Phillipp) <--- IMU data (Sam, Nodes 2-5)
                                    |
                                    v
                              Pose Estimate --> TinyML classifier (Vedant, Week 5)
```

## Author

**Francisco Benitez** — Camera Engineer, Team DRIFT
ECE 56800, Purdue University, Spring 2026
