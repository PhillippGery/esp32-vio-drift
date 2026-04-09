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
 * @brief Send odometry packet over UDP.
 * @param timestamp_ms Current timestamp in milliseconds.
 */
void sendOdometryPacket(uint32_t timestamp_ms);

} // namespace drift