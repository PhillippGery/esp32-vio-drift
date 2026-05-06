/**
 * @file    transport.h
 * @brief   WiFi UDP transport and JSON packet serialization for PROJECT DRIFT
 *
 * @author  Sam (Firmware)
 */

#pragma once
#include <Arduino.h>

namespace drift {

/**
 * @brief Initialize WiFi connection and UDP socket.
 * @return true if successful, false on failure.
 */
bool transportInit();

/**
 * @brief Send IMU packet over UDP.
 * @param timestamp_ms Current timestamp in milliseconds.
 * @param ax Accelerometer X
 * @param ay Accelerometer Y
 * @param az Accelerometer Z
 * @param gx Gyroscope X
 * @param gy Gyroscope Y
 * @param gz Gyroscope Z
 */
void sendImuPacket(uint32_t timestamp_ms, float ax, float ay, float az, float gx, float gy, float gz, float temp_c);

} // namespace drift