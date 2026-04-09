/**
 * @file    transport.h
 * @brief   WiFi UDP transport and JSON packet serialization for PROJECT DRIFT
 *
 * @author  Sam (Firmware)
 */

 // Note: PlatformIO automatically includes: all .cpp sources under src/ and all headers under include/. So the new transport.cpp and transport.h files are already part of the build without any platformio.ini changes. Thus, no platformio.ini edits are required. The projects will compile the new transport module as long as transport.cpp is in src/, transport.h is in include/, and main.cpp includes "transport.h".

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