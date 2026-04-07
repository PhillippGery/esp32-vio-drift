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

namespace drift {

void ekfInit() {
    // 1. Zero out the initial state
    x.Fill(0.0f);

    // 2. Initialize Covariance Matrix (P)
    P.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) {
        P(i, i) = 1.0f; // Start with baseline uncertainty
    }

    // 3. Initialize Process Noise (Q)
    Q.Fill(0.0f);
    // Tune these values based on your specific MPU-6050's noise characteristics
    Q(0, 0) = 0.001f; // px noise
    Q(1, 1) = 0.001f; // py noise
    Q(2, 2) = 0.01f;  // vx noise
    Q(3, 3) = 0.01f;  // vy noise
    Q(4, 4) = 0.005f; // yaw noise
    Q(5, 5) = 0.0001f; // gyro bias noise (changes very slowly)
}

void ekfPredict(float ax, float ay, float gz, float dt) {
    // 1. Extract current state variables
    float px = x(0), py = x(1);
    float vx = x(2), vy = x(3);
    float yaw = x(4), bgz = x(5);

    // 2. Correct raw gyro data using our current bias estimate
    float wz = gz - bgz;
    float yaw_new = yaw + wz * dt;

    // 3. Trigonometry for the 2D Rotation Matrix
    float c = cos(yaw);
    float s = sin(yaw);

    // 4. Rotate Acceleration to Global Frame
    // WARNING: ax and ay MUST have the static calibration offset removed before arriving here!
    float a_global_x = ax * c - ay * s;
    float a_global_y = ax * s + ay * c;

    // 5. Update State Vector (x)
    x(0) = px + vx * dt;                 // p_x
    x(1) = py + vy * dt;                 // p_y
    x(2) = vx + a_global_x * dt;         // v_x
    x(3) = vy + a_global_y * dt;         // v_y
    x(4) = yaw_new;                      // yaw
    x(5) = bgz;                          // bias stays constant in predict

    // 6. Build the Jacobian Matrix (F_mat)
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

    // 7. Update the Covariance Matrix (P = F * P * F^T + Q)
    P = F_mat * P * (~F_mat) + Q;
}

// Prepped for the forward-facing Essential Matrix decomposition output
void ekfUpdateCamera(float delta_yaw_cam, float t_x, float t_y, float confidence) {
    // TODO: Kalman Gain (K) math.
    // 1. Calculate measurement residual (y = z - Hx)
    // 2. Calculate measurement covariance (S = H * P * H^T + R)
    // 3. Calculate Kalman Gain (K = P * H^T * S^-1)
    // 4. Update State (x = x + K * y)
    // 5. Update Covariance (P = (I - K * H) * P)
    
    // NOTE: t_x and t_y are a unit vector. You will fuse this with 
    // the direction of x(2) and x(3) to correct the trajectory, 
    // while keeping the magnitude from the IMU integration.
}

void ekfGetPosition(float &px, float &py) {
    px = x(0);
    py = x(1);
}

void ekfGetOrientation(float &yaw) {
    yaw = x(4);
}

} // namespace drift