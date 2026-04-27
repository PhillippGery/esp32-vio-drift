#include "dead_reckoning.h"

namespace drift {

DeadReckoner::DeadReckoner() : px(0.0f), py(0.0f), vx(0.0f), vy(0.0f), yaw(0.0f) {}

void DeadReckoner::update(float ax, float ay, float gz, float dt) {
    // Integrate Gyro for Yaw (Orientation)
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