# API Documentation

Auto-generate from source with Doxygen, or maintain manually here.

## Generating Doxygen Docs

```bash
doxygen Doxyfile
# Output: 20_docs/api/html/index.html
```

## Key Modules

| Module | File | Description |
|---|---|---|
| EKF | `10_src/node1_vio/include/ekf.h` | Extended Kalman Filter state machine |
| IMU Driver | `include/imu_driver.h` | MPU-6050 I²C read/write |
| Camera Driver | `include/cam_driver.h` | ArduCAM SPI frame capture (Node 1) |
| Transport | `include/transport.h` | WiFi UDP JSON packet transmit/receive |
