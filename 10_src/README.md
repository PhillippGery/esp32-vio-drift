# 10_src — Firmware Source

PlatformIO projects for all five nodes. Open each sub-folder as a separate
PlatformIO project in VS Code.

| Folder | Node | Hardware |
|---|---|---|
| `node1_vio/` | Node 1 | ESP32 Feather V2 + MPU-6050 + ESP32S3 Sense |
| `node2_imu/` | Node 2 | ESP32 Feather V2 + MPU-6050 |
| `node3_5_imu/` | Nodes 3–5 | ESP32 Feather V2 + MPU-6050 (shared firmware) |

## Flash a specific node

First:
esptool.py --chip esp32 --port /dev/cu.usbserial-5A6C0403221 erase_flash

```bash
# Node 1 (VIO)
cd node1_vio && pio run --target upload

# Node 2
cd node2_imu && pio run --target upload

# Node 3, 4, or 5 (choose env)
cd node3_5_imu && pio run -e node3 --target upload
```

## Potential issues

The issue for me was that PlatformIO's default upload speed (921600 baud) caused a serial handshake failure. Using esptool directly at 460800 baud worked perfectly.

To test:
python data_pipeline/capture_udp.py --duration 60 --output ../30_data/raw_imu/