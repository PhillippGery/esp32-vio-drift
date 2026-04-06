#include "ekf.h"
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// ─── Core EKF Matrices ───────────────────────────────────────────────────
// State: [px, py, pz, vx, vy, vz, roll, pitch, yaw, bx, by, bz]^T
static Matrix<drift::STATE_DIM, 1> x;

// State Covariance
static Matrix<drift::STATE_DIM, drift::STATE_DIM> P;

// Process Noise Covariance
static Matrix<drift::STATE_DIM, drift::STATE_DIM> Q;

namespace drift {

void ekfInit() {
    // 1. Zero out the initial state
    x.Fill(0.0f);

    // 2. Initialize Covariance Matrix (P)
    // We start with some uncertainty. 
    // Diagonal matrix: independent variance for each state variable.
    P.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) {
        P(i, i) = 1.0f; // Start with 1.0 variance (tune this later)
    }

    // 3. Initialize Process Noise (Q)
    // This dictates how much drift we expect per time step.
    Q.Fill(0.0f);
    for (int i = 0; i < STATE_DIM; i++) {
        Q(i, i) = 0.01f; // Small noise base value (tune this later)
    }
}

void ekfPredict(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    // 1. Extract current state variables for readability
    float roll = x(6), pitch = x(7), yaw = x(8);
    float bx = x(9), by = x(10), bz = x(11);

    // 2. Correct raw gyro data using our current bias estimate
    float wx = gx - bx;
    float wy = gy - by;
    float wz = gz - bz;

    // 3. Update Orientation (Euler Integration)
    roll  += wx * dt;
    pitch += wy * dt;
    yaw   += wz * dt;

    // 4. Build the Rotation Matrix (Body Frame -> Global Frame)
    // This requires calculating the sines and cosines of our new orientation
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw), sy = sin(yaw);

    Matrix<3, 3> R = {
        cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy,
        cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy,
        -sp,    sr*cp,             cr*cp
    };

    // 5. Rotate Acceleration and Subtract Gravity
    Matrix<3, 1> a_meas = {ax, ay, az};
    Matrix<3, 1> a_global = R * a_meas;
    Matrix<3, 1> gravity = {0.0f, 0.0f, 9.81f}; // Standard gravity in m/s^2
    Matrix<3, 1> a_true = a_global - gravity;

    // 6. Update Position (p = p + v*dt)
    x(0) += x(3) * dt;
    x(1) += x(4) * dt;
    x(2) += x(5) * dt;

    // 7. Update Velocity (v = v + a*dt)
    x(3) += a_true(0) * dt;
    x(4) += a_true(1) * dt;
    x(5) += a_true(2) * dt;

    // 8. Save updated orientation back to the state vector
    x(6) = roll; 
    x(7) = pitch; 
    x(8) = yaw;

    // 9. UPDATE UNCERTAINTY (Covariance P)
    // The equation is: P = F * P * F^T + Q
    // We need the Jacobian matrix (F) of the state transition equations above.
    
    Matrix<STATE_DIM, STATE_DIM> F_mat;
    F_mat.Fill(0.0f);
    for(int i = 0; i < STATE_DIM; i++) {
        F_mat(i, i) = 1.0f; // TODO: Placeholder! 
    }

    // ~F_mat is the transpose operator in the BLA library
    P = F_mat * P * (~F_mat) + Q;
}

void ekfUpdateCamera(float du, float dv) {
    // Visual odometry update math goes here.
}

void ekfGetPosition(float &px, float &py, float &pz) {
    px = x(0);
    py = x(1);
    pz = x(2);
}

void ekfGetOrientation(float &roll, float &pitch, float &yaw) {
    roll  = x(6);
    pitch = x(7);
    yaw   = x(8);
}

} // namespace drift