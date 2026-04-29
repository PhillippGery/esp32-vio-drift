#pragma once

namespace drift {

// ─── The New 2D Planar Architecture ───
// State Vector: [px, py, vx, vy, yaw, bgz]^T
constexpr int STATE_DIM = 6;

/**
 * @brief Initializes the EKF matrices (State, Covariance, Process Noise).
 */
void ekfInit();

// ─── The Tuning Payload ──────────────────────────────────────────────────
struct EkfConfig {
    // Initial Covariance (P)
    float p_init = 1.0f;
    
    // Process Noise (Q) - How much drift we expect per step
    float q_px  = 0.001f;
    float q_py  = 0.001f;
    float q_vx  = 0.01f;
    float q_vy  = 0.01f;
    float q_yaw = 0.005f;
    float q_bgz = 0.0001f;
    
    // Measurement Noise (R) - Camera baseline error
    float cam_base_noise = 0.001f;
};


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

/**
 * @brief Retrieves the current gyro bias estimate (rad/s). Use for diagnostics
 * to verify ZUPT is converging — should approach the residual thermal offset.
 */
void ekfGetBias(float &bgz);


/**
 * @brief Forces the state vector back to absolute zero and resets uncertainty.
 * Use this after hardware initialization or when recovering from a crash.
 */
void ekfReset();

/**
 * @brief Zero-velocity / Zero-rate update (ZUPT).
 * Call when the robot is confirmed stationary. Directly corrects the gyro
 * bias state (bgz) and drives velocity to zero. This is the primary mechanism
 * for compensating thermal gyro drift — when still, gz = bgz (true rate = 0).
 * @param gz The current filtered gz reading in rad/s.
 */
void ekfZupt(float gz);

} // namespace drift