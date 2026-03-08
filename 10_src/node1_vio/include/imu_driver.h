/**
 * @file  imu_driver.h
 * @brief MPU-6050 I²C driver interface — Node 1 (VIO)
 *
 * @author Sam
 */

#pragma once
#include <Arduino.h>

namespace drift {

/**
 * @brief Initialize MPU-6050 over I²C.
 * @return true if device acknowledged, false on failure.
 */
bool imuInit();

/**
 * @brief Read raw 6-axis data and convert to SI units.
 * @param[out] ax  Acceleration X (m/s²)
 * @param[out] ay  Acceleration Y (m/s²)
 * @param[out] az  Acceleration Z (m/s²)
 * @param[out] gx  Angular rate X (rad/s)
 * @param[out] gy  Angular rate Y (rad/s)
 * @param[out] gz  Angular rate Z (rad/s)
 * @return true on successful I²C read.
 */
bool imuRead(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);

/**
 * @brief Read MPU-6050 internal temperature (for bias tracking).
 * @return Temperature in °C.
 */
float imuReadTemp();

} // namespace drift
