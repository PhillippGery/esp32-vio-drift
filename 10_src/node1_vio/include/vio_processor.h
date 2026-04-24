#pragma once

#include "optical_flow.h" 

namespace drift {

// The clean, metric payload we will hand to the Kalman Filter
struct VioMeasurement {
    float vx;         // Forward/Backward velocity (m/s)
    float vy;         // Left/Right strafing velocity (m/s)
    float confidence; // 0.0 to 1.0 (How many features survived the filter)
    bool valid;       // True if the frame had enough points to trust
};

/**
 * @brief Converts raw pixel flow into absolute metric velocity using Inverse Perspective Mapping.
 * @param flow_vectors The array output from Pancho's Lucas-Kanade function.
 * @param count The number of tracked features in the array.
 * @param dt Time in seconds since the last camera frame.
 * @return VioMeasurement containing the filtered real-world velocity.
 */
VioMeasurement processFlowToMetric(const FlowVector* flow_vectors, int count, float dt, float delta_yaw_imu);

} // namespace drift