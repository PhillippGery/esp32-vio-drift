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
    float q_yaw = 0.0005f; // lower = trust IMU yaw integration more, less camera pull
    float q_bgz = 0.0001f;
    
    // Measurement Noise (R) - Camera baseline error
    float cam_base_noise = 0.001f;

    // ZUPT Measurement Noise (R) - stationary update
    float zupt_r_vel  = 0.01f;   // velocity channels (vx, vy) when stationary
    float zupt_r_bgz  = 0.01f;   // gyro bias channel — wider = smoother convergence
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
 * @brief Zero-velocity update (velocity channels only, no bgz).
 * Use during the 300–1500 ms window after motion stops, before the
 * yaw–bgz cross-covariance has had time to decay.
 */
void ekfZuptVelocity();

/**
 * @brief Full Zero-velocity / Zero-rate update (ZUPT).
 * Call when the robot is confirmed stationary for >1500 ms.
 * @param gz The current filtered gz reading in rad/s.
 */
void ekfZupt(float gz);

/**
 * @brief Sets the initial gyro bias estimate in the state vector.
 * Call this once at startup after the first few seconds of calibration.
 * @param bgz_rad_s Initial bias estimate in radians per second.
 */
void ekfSetInitialBias(float bgz_rad_s);

/**
 * @brief Zeroes the motion-history cross-covariances between yaw and
 * {vx, vy, bgz} in P. Call exactly once when the robot transitions from
 * motion to rest, just before the first ZUPT fires.
 *
 * These cross-covariances accumulate during rotation and cause subsequent
 * ZUPT corrections to retroactively corrupt the correctly integrated yaw.
 * Zeroing them tells the filter: "the past yaw integral was correct;
 * bgz/velocity corrections should only affect future integration."
 */
void ekfDecoupleOnStop();

} // namespace drift