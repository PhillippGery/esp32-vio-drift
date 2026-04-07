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
 * @brief Update step: Anchors the physics engine using the forward-facing camera.
 * @param delta_yaw_cam The absolute yaw change calculated by the Essential Matrix.
 * @param t_x The X component of the normalized translation unit vector.
 * @param t_y The Y component of the normalized translation unit vector.
 * @param confidence Reliability metric to dynamically scale the Measurement Noise (R).
 */
void ekfUpdateCamera(float delta_yaw_cam, float t_x, float t_y, float confidence);

/**
 * @brief Retrieves the current 2D position estimate for telemetry.
 */
void ekfGetPosition(float &px, float &py);

/**
 * @brief Retrieves the current orientation estimate for telemetry.
 */
void ekfGetOrientation(float &yaw);

} // namespace drift