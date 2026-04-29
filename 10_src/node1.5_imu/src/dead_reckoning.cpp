#include "dead_reckoning.h"

namespace drift {

DeadReckoner::DeadReckoner() : px(0.0f), py(0.0f), vx(0.0f), vy(0.0f), yaw(0.0f), gz_bias(0.0f) {}

void DeadReckoner::update(float ax, float ay, float gz, float dt) {
    // Accelerometer-confirmed ZUPT (Zero-rate Update):
    // When both gyro and horizontal acceleration are below threshold, the robot
    // is stationary → gz equals the current thermal bias, not real rotation.
    // Fast EMA (τ ≈ 0.24s at 208Hz) tracks thermal ramp within seconds.
    // Hard zero prevents ANY noise from integrating into yaw while still.
    float acc_h = sqrtf(ax * ax + ay * ay);
    bool is_stationary = (acc_h < 0.3f) && (fabsf(gz) < 0.008f);

    if (is_stationary) {
        gz_bias += (gz - gz_bias) * 0.02f; // fast convergence when confirmed still
        gz = 0.0f;                          // hard zero: no integration while stationary
    } else {
        gz -= gz_bias;                      // subtract tracked bias during motion
    }

    yaw += gz * dt;

    // Build the 2D Rotation Matrix (Local to Global Frame)
    float c = cos(yaw);
    float s = sin(yaw);

    // Rotate the acceleration into the global navigation frame
    float a_global_x = ax * c - ay * s;
    float a_global_y = ax * s + ay * c;

    // The Flawed Double Integration (Where the error compounds)
    vx += a_global_x * dt;
    vy += a_global_y * dt;

    px += vx * dt;
    py += vy * dt;
}

float DeadReckoner::getDeltaMovement() {
    // Pythagorean theorem: a^2 + b^2 = c^2
    return sqrt((px * px) + (py * py));
}

} // namespace drift