#pragma once

namespace drift {

// ─── The New 2D Planar Architecture ───
// State Vector: [px, py, vx, vy, yaw, bgz]^T
constexpr int STATE_DIM = 6;

/**
 * @brief Initializes the EKF matrices (State, Covariance, Process Noise).
 */
void ekfInit();

/**
 * @brief Predict step: Drives the physics engine forward using IMU data.
 * @param ax Linear acceleration in X (m/s^2) - MUST be static-bias corrected!
 * @param ay Linear acceleration in Y (m/s^2) - MUST be static-bias corrected!
 * @param gz Angular velocity around Z (rad/s) - MUST be in radians!
 * @param dt Time delta since last reading (seconds)
 */
void ekfPredict(float ax, float ay, float gz, float dt);

/**
 * @brief Update step: Anchors the physics engine using camera data.
 * @param meas_vx The measured X velocity (m/s). 0.0 for static test.
 * @param meas_vy The measured Y velocity (m/s). 0.0 for static test.
 * @param meas_yaw The absolute measured yaw (radians).
 * @param confidence Reliability metric (0.0 to 1.0). High value = trust camera.
 */
void ekfUpdateCamera(float meas_vx, float meas_vy, float meas_yaw, float confidence);

/**
 * @brief Retrieves the current 2D position estimate for telemetry.
 */
void ekfGetPosition(float &px, float &py);

/**
 * @brief Retrieves the current orientation estimate for telemetry.
 */
void ekfGetOrientation(float &yaw);

} // namespace drift