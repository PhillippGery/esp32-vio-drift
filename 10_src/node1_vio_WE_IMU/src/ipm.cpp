#include "ipm.h"
#include <math.h>

namespace drift {

// Pre-calculate the camera pitch in radians to save CPU cycles
constexpr float CAMERA_PITCH_RAD = CAMERA_PITCH_DEG * (M_PI / 180.0f);

GroundPoint pixelToGround(float u, float v) {
    GroundPoint pt;
    pt.valid = false;
    pt.x = 0.0f;
    pt.y = 0.0f;

    // 1. Calculate the vertical angle of the pixel relative to the camera's optical axis
    // In an image, v=0 is the top. We invert it so positive angles look down.
    float delta_v = v - IMAGE_CENTER_Y; 
    float alpha = atanf(delta_v / FOCAL_LENGTH_Y);

    // 2. Calculate the total angle of the ray relative to the horizon
    // Pitch is the mechanical tilt. Alpha is the pixel's offset from the center.
    float total_pitch = CAMERA_PITCH_RAD + alpha;

    // If the ray is pointing at or above the horizon, it never hits the floor. Reject it.
    if (total_pitch <= 0.01f) {
        return pt; 
    }

    // 3. Calculate forward distance (Y) on the ground using geometry
    // tan(angle) = opposite / adjacent -> tan(total_pitch) = height / distance
    float distance_y = CAMERA_HEIGHT_M / tanf(total_pitch);

    // 4. Calculate lateral angle (beta) and lateral distance (X)
    float delta_u = u - IMAGE_CENTER_X;
    float beta = atanf(delta_u / FOCAL_LENGTH_X);
    
    // The lateral distance spreads out the further away the point is
    // We calculate the ray length from the lens to the ground point first
    float ray_length_ground = sqrtf((CAMERA_HEIGHT_M * CAMERA_HEIGHT_M) + (distance_y * distance_y));
    float distance_x = ray_length_ground * tanf(beta);

    pt.x = distance_x;
    pt.y = distance_y;
    pt.valid = true;

    return pt;
}

} // namespace drift