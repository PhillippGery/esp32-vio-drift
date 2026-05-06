#include "ekf.h"
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// ─── Core EKF Matrices (6-DOF Planar) ────────────────────────────────────
// State: [px, py, vx, vy, yaw, bgz]^T
static Matrix<drift::STATE_DIM, 1> x;

// State Covariance (Uncertainty)
static Matrix<drift::STATE_DIM, drift::STATE_DIM> P;

// Process Noise Covariance (How much we drift per step)
static Matrix<drift::STATE_DIM, drift::STATE_DIM> Q;

static drift::EkfConfig active_config;

namespace drift {

void ekfInit() {
    //Zero out the initial state
    x.Fill(0.0f);

    // Initialize Covariance Matrix (P)
    P.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) {
        P(i, i) = active_config.p_init; // Start with baseline uncertainty
    }

    // Initialize Process Noise (Q)
    Q.Fill(0.0f);
    // Tune these values based on your specific MPU-6050's noise characteristics
    Q(0, 0) = active_config.q_px; // px noise
    Q(1, 1) = active_config.q_py; // py noise
    Q(2, 2) = active_config.q_vx;  // vx noise
    Q(3, 3) = active_config.q_vy;  // vy noise
    Q(4, 4) = active_config.q_yaw; // yaw noise
    Q(5, 5) = active_config.q_bgz; // gyro bias noise (changes very slowly)
}

void ekfPredict(float ax, float ay, float gz, float dt) {
    // Extract current state variables
    float px = x(0), py = x(1);
    float vx = x(2), vy = x(3);
    float yaw = x(4), bgz = x(5);

    // Correct raw gyro data using our current bias estimate
    float wz = gz - bgz;
    float yaw_new = yaw + wz * dt;

    // Trigonometry for the 2D Rotation Matrix
    float c = cos(yaw);
    float s = sin(yaw);

    // Rotate Acceleration to Global Frame

    float a_global_x = ax * c - ay * s;
    float a_global_y = ax * s + ay * c;

    // Update State Vector (x)
    x(0) = px + vx * dt;                 // p_x
    x(1) = py + vy * dt;                 // p_y
    x(2) = vx + a_global_x * dt;         // v_x
    x(3) = vy + a_global_y * dt;         // v_y
    x(4) = yaw_new;                      // yaw
    x(5) = bgz;                          // bias stays constant in predict

    // Build the Jacobian Matrix (F_mat)
    Matrix<STATE_DIM, STATE_DIM> F_mat;
    F_mat.Fill(0.0f);
    
    // Set diagonal to 1.0
    for(int i = 0; i < STATE_DIM; i++) {
        F_mat(i, i) = 1.0f;
    }

    // The Non-Linear Partial Derivatives
    F_mat(0, 2) = dt; // d(p_x) / d(v_x)
    F_mat(1, 3) = dt; // d(p_y) / d(v_y)
    
    // How yaw orientation affects global velocity integration
    F_mat(2, 4) = (-ax * s - ay * c) * dt; // d(v_x) / d(yaw)
    F_mat(3, 4) = (ax * c - ay * s) * dt;  // d(v_y) / d(yaw)
    
    // How gyro bias affects the yaw estimate
    F_mat(4, 5) = -dt; // d(yaw) / d(bgz)

    // Update the Covariance Matrix (P = F * P * F^T + Q)
    P = F_mat * P * (~F_mat) + Q;
}

void ekfSetInitialBias(float bgz_rad_s) {
    x(5) = bgz_rad_s;
    P(5, 5) = 1e-6f;  // tell the filter "we're confident in this value"
}


void ekfUpdateCamera(float meas_vx, float meas_vy, float meas_yaw, float confidence) {
    // Define Measurement Vector (z)
    Matrix<3, 1> z = {meas_vx, meas_vy, meas_yaw};

    // Define Observation Matrix (H)
    // We are observing vx (index 2), vy (index 3), and yaw (index 4)
    Matrix<3, STATE_DIM> H = {
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f
    };

    // Define Measurement Noise Covariance (R)
    // High confidence = tiny noise = massive Kalman Gain.
    float scaled_noise = active_config.cam_base_noise / (confidence + 0.00001f); // Avoid divide by zero
    
    Matrix<3, 3> R = {
        scaled_noise, 0.0f, 0.0f,
        0.0f, scaled_noise, 0.0f,
        0.0f, 0.0f, scaled_noise
    };

    // Calculate Innovation (y = z - Hx)
    Matrix<3, 1> y = z - H * x;

    // Normalize the yaw error so the robot doesn't do a 360-degree spin to correct 1 degree
    while (y(2) > PI) y(2) -= 2.0f * PI;
    while (y(2) < -PI) y(2) += 2.0f * PI;

    // Calculate Innovation Covariance (S = H * P * H^T + R)
    Matrix<3, 3> S = H * P * (~H) + R;

    // Calculate Kalman Gain (K = P * H^T * S^-1)
    Matrix<3, 3> S_inv;
    bool is_nonsingular = Invert(S, S_inv);
    if (!is_nonsingular) {
        return; // Matrix singular, skip update to prevent hard fault
    }
    
    Matrix<STATE_DIM, 3> K = P * (~H) * S_inv;

    // Apply the Correction to the State (x = x + Ky)
    x += K * y;

    // Reduce the Uncertainty (P = (I - KH) * P)
    Matrix<STATE_DIM, STATE_DIM> I;
    I.Fill(0.0f);
    for(int i = 0; i < STATE_DIM; i++) {
        I(i, i) = 1.0f;
    }

    P = (I - K * H) * P;
}

void ekfGetPosition(float &px, float &py) {
    px = x(0);
    py = x(1);
}

void ekfGetOrientation(float &yaw) {
    yaw = x(4);
}

void ekfGetBias(float &bgz) {
    bgz = x(5);
}

void ekfReset() {
    x.Fill(0.0f);
    P.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) {
        P(i, i) = active_config.p_init;
    }
}

void ekfZuptVelocity() {
    Matrix<2, 1> z = {0.0f, 0.0f};
    Matrix<2, STATE_DIM> H;
    H.Fill(0.0f);
    H(0, 2) = 1.0f; // vx
    H(1, 3) = 1.0f; // vy

    Matrix<2, 2> R;
    R.Fill(0.0f);
    R(0, 0) = active_config.zupt_r_vel;
    R(1, 1) = active_config.zupt_r_vel;

    Matrix<2, 1> y = z - H * x;
    Matrix<2, 2> S = H * P * (~H) + R;
    Matrix<2, 2> S_inv;
    if (!Invert(S, S_inv)) return;

    Matrix<STATE_DIM, 2> K = P * (~H) * S_inv;
    x += K * y;

    Matrix<STATE_DIM, STATE_DIM> I;
    I.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) I(i, i) = 1.0f;
    P = (I - K * H) * P;
}

void ekfDecoupleOnStop() {
    // Break the yaw cross-covariances that accumulate during rotation.
    // These couple post-stop ZUPT corrections back into the already-correct
    // integrated yaw. Called once at the motion→rest transition.
    P(4, 2) = P(2, 4) = 0.0f; // yaw ↔ vx
    P(4, 3) = P(3, 4) = 0.0f; // yaw ↔ vy
    P(4, 5) = P(5, 4) = 0.0f; // yaw ↔ bgz
}

void ekfZupt(float gz) {
    // ZUPT: robot is stationary → true rotation rate = 0 → gz IS the thermal bias.
    // Measurement vector: [vx=0, vy=0, bgz=gz]
    // H observes state indices [2=vx, 3=vy, 5=bgz] directly.
    Matrix<3, 1> z = {0.0f, 0.0f, gz};

    Matrix<3, STATE_DIM> H;
    H.Fill(0.0f);
    H(0, 2) = 1.0f; // vx
    H(1, 3) = 1.0f; // vy
    H(2, 5) = 1.0f; // bgz ← this column was never observed in ekfUpdateCamera

    Matrix<3, 3> R;
    R.Fill(0.0f);
    R(0, 0) = active_config.zupt_r_vel;
    R(1, 1) = active_config.zupt_r_vel;
    R(2, 2) = active_config.zupt_r_bgz;

    Matrix<3, 1> y = z - H * x;

    Matrix<3, 3> S = H * P * (~H) + R;
    Matrix<3, 3> S_inv;
    if (!Invert(S, S_inv)) return;

    Matrix<STATE_DIM, 3> K = P * (~H) * S_inv;
    x += K * y;

    Matrix<STATE_DIM, STATE_DIM> I;
    I.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) I(i, i) = 1.0f;
    P = (I - K * H) * P;
}

} // namespace drift