# 30_data — Sensor Logs & ML Datasets

> **Large files must use Git LFS.**
> Run `git lfs track "*.csv" "*.bin" "*.zip"` before committing data files.

| Folder | Contents |
|---|---|
| `raw_imu/` | Raw CSV/binary logs streamed from all 5 nodes |
| `labeled_datasets/` | Manually labeled motion segments (for TinyML training) |
| `edgeimpulse_export/` | Dataset ZIP exports from Edge Impulse |

## File Naming Convention

```
raw_imu/
  node{N}_YYYYMMDD_HHMMSS.csv    ← e.g. node1_20260315_143022.csv
  node{N}_YYYYMMDD_HHMMSS.bin    ← binary format (optional)

labeled_datasets/
  {motion_class}_YYYYMMDD.csv    ← e.g. walking_20260315.csv
```

## CSV Schema — Raw IMU Log

```
timestamp_ms, node_id, ax, ay, az, gx, gy, gz, temp_c
```
