# Calibration Procedures

## IMU Calibration (MPU-6050)

Run the calibration utility from `60_scripts/utils/imu_calibrate.py` with the
node placed **flat and stationary** on a level surface.

The script collects 1000 samples at rest and computes:
- Accelerometer bias (x, y, z) in m/s²
- Gyroscope bias (x, y, z) in rad/s

Output is saved to `30_data/raw_imu/calibration_nodeN.json`.

### Accepted tolerances
| Parameter | Target | Max deviation |
|---|---|---|
| Accel Z at rest | 9.81 m/s² | ± 0.05 m/s² |
| Gyro bias | 0 rad/s | ± 0.002 rad/s |

## Camera Calibration (ArduCAM OV2640)

Use OpenCV's `calibrateCamera()` with a 9×6 checkerboard pattern.
Collect ≥ 20 images from different angles and distances.

Output: intrinsic matrix K and distortion coefficients saved to
`30_data/calibration/camera_intrinsics.json`.

```bash
python 60_scripts/utils/camera_calibrate.py \
    --images 30_data/raw_imu/calib_frames/ \
    --output 30_data/calibration/camera_intrinsics.json
```
