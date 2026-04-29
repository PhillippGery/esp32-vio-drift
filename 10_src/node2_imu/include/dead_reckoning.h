#pragma once
#include <math.h>

namespace drift {

class DeadReckoner {
public:
    // ─── State Variables ───
    float px, py;    // Position (meters)
    float vx, vy;    // Velocity (m/s)
    float yaw;       // Orientation (radians)
    float gz_bias;   // Online gyro bias estimate (rad/s)

    // Constructor guarantees we start at zero
    DeadReckoner();

    /**
     * @brief Integrates raw IMU data to estimate 2D position.
     * @param ax Accel X (m/s²) - MUST have static bias removed!
     * @param ay Accel Y (m/s²) - MUST have static bias removed!
     * @param gz Gyro Z (rad/s) - MUST be in radians!
     * @param dt Time delta since last reading (seconds)
     */
    void update(float ax, float ay, float gz, float dt);

    /**
     * @brief Calculates the total straight-line distance from the origin (0,0).
     * @return Euclidean distance in meters.
     */
    float getDeltaMovement();
};

} // namespace drift