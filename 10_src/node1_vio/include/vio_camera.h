#pragma once

namespace drift {

/**
 * @brief Initializes the camera peripheral and allocates PSRAM buffers.
 * @return true on success, false on failure.
 */
bool cameraInit();

/**
 * @brief Captures a frame, runs FAST+LK, and accumulates flow.
 * @param[out] dx         Accumulated X displacement in meters
 * @param[out] dy         Accumulated Y displacement in meters
 * @param[out] confidence How reliable the measurement is
 * @return true ONLY if a full window of flow has accumulated and VIO data is ready.
 */
bool cameraProcessFrame(float &vx, float &vy, float &confidence, float dt, float delta_yaw_imu);

/**
 * @brief Checks Serial for the 'c' command to trigger a debug capture.
 */
void cameraDebugCheck();

} // namespace drift