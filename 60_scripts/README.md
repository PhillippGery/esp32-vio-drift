# 60_scripts — Host-Side Python Tooling

Python 3.9+ required. Install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

| Folder | Owner | Contents |
|---|---|---|
| `data_pipeline/` | Sam / Phillipp | Serial & WiFi data capture scripts |
| `visualization/` | Jack | Real-time dashboard, drift plots |
| `utils/` | All | IMU calibration, log parsers, converters |

## Quick Start

```bash
# Capture all 5 nodes simultaneously over WiFi UDP
python 60_scripts/data_pipeline/capture_udp.py --duration 60 --output 30_data/raw_imu/

# Launch Jack's real-time dashboard
python 60_scripts/visualization/dashboard.py

# Run IMU calibration utility
python 60_scripts/utils/imu_calibrate.py --port /dev/ttyUSB0 --node 1
```
