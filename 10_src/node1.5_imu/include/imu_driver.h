/**
 * @file  imu_driver.h
 * @brief MPU-6050 I²C driver interface — Nodes 2–5 (IMU only)
 *
 * @author Sam
 */

#pragma once
#include <Arduino.h>


namespace drift {

bool imuInit();
bool imuRead(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);
float imuReadTemp();

} // namespace drift
