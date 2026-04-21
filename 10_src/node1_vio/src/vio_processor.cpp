#include "vio_processor.h"
#include "ipm.h"
#include <math.h> // Needed for cosf and sinf
#include <algorithm> // Required for std::sort

constexpr int MAX_FEATURES = 100; 

namespace drift {

// NOTICE: Added delta_yaw_imu to the signature!
VioMeasurement processFlowToMetric(const FlowVector* flow_vectors, int count, float dt, float delta_yaw_imu) {
    VioMeasurement result = {0.0f, 0.0f, 0.0f, false};

    if (count == 0 || dt <= 0.0001f) return result;

    float vx_array[MAX_FEATURES];
    float vy_array[MAX_FEATURES];
    int valid_points = 0;

    // Pre-calculate sine and cosine of the IMU yaw change for the whole frame
    float c = cosf(delta_yaw_imu);
    float s = sinf(delta_yaw_imu);

    for (int i = 0; i < count && i < MAX_FEATURES; i++) {
        if (!flow_vectors[i].valid) continue;

        // 1. Project the OLD pixel coordinate to the ground
        GroundPoint p_old = pixelToGround(flow_vectors[i].px, flow_vectors[i].py);
        
        // 2. THE FIX: Derotate the old point using the IMU's angle change
        float p_old_rot_x = p_old.x * c - p_old.y * s;
        float p_old_rot_y = p_old.x * s + p_old.y * c;

        // 3. Project the NEW pixel coordinate to the ground
        GroundPoint p_new = pixelToGround(flow_vectors[i].px + flow_vectors[i].dx, 
                                          flow_vectors[i].py + flow_vectors[i].dy);

        // 4. Convert to metric velocity using the derotated old point!
        if (p_old.valid && p_new.valid) {
            vx_array[valid_points] = (p_old_rot_x - p_new.x) / dt;
            vy_array[valid_points] = (p_old_rot_y - p_new.y) / dt;
            valid_points++;
        }
    }

    if (valid_points < 3) {
        return result; 
    }

    std::sort(vx_array, vx_array + valid_points);
    std::sort(vy_array, vy_array + valid_points);

    result.vx = vx_array[valid_points / 2];
    result.vy = vy_array[valid_points / 2];

    result.confidence = (float)valid_points / (float)count;
    result.valid = true;

    return result;
}

} // namespace drift