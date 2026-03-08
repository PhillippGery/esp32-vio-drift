/**
 * @file  ekf.h
 * @brief Extended Kalman Filter — state prediction & update interface
 *
 * State vector (12-DOF):
 *   [px, py, pz,        position (m)
 *    vx, vy, vz,        velocity (m/s)
 *    roll, pitch, yaw,  orientation (rad)
 *    bx, by, bz]        gyro bias (rad/s)
 *
 * @author Phillipp Gery
 */

#pragma once
#include <Arduino.h>

namespace drift {

/** @brief EKF state dimension */
constexpr int STATE_DIM = 12;

/** @brief EKF measurement dimension (IMU: ax,ay,az,gx,gy,gz) */
constexpr int IMU_MEAS_DIM = 6;

/**
 * @brief Initialize EKF with identity covariance matrices.
 */
void ekfInit();

/**
 * @brief IMU predict step — propagate state using gyro/accel.
 * @param ax  Accel X (m/s²)
 * @param ay  Accel Y (m/s²)
 * @param az  Accel Z (m/s²)
 * @param gx  Gyro X (rad/s)
 * @param gy  Gyro Y (rad/s)
 * @param gz  Gyro Z (rad/s)
 * @param dt  Time step (s)
 */
void ekfPredict(float ax, float ay, float az,
                float gx, float gy, float gz,
                float dt);

/**
 * @brief Camera update step — correct state with visual feature displacement.
 * @param du  Optical flow u (pixels)
 * @param dv  Optical flow v (pixels)
 */
void ekfUpdateCamera(float du, float dv);

/**
 * @brief Get current position estimate.
 * @param[out] px  X position (m)
 * @param[out] py  Y position (m)
 * @param[out] pz  Z position (m)
 */
void ekfGetPosition(float &px, float &py, float &pz);

/**
 * @brief Get current orientation (Euler angles).
 * @param[out] roll   Roll (rad)
 * @param[out] pitch  Pitch (rad)
 * @param[out] yaw    Yaw (rad)
 */
void ekfGetOrientation(float &roll, float &pitch, float &yaw);

} // namespace drift
