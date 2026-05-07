#pragma once

#include <stdint.h>

namespace drift {

// ─── CAMERA HARDWARE CONSTANTS ──────────────────────────────────────────

// Height of the camera lens center from the floor (in meters)
constexpr float CAMERA_HEIGHT_M = 0.07f; 

// Downward tilt angle of the camera (in degrees). 
// 0 = looking straight at the horizon. 90 = looking straight down at the floor.
constexpr float CAMERA_PITCH_DEG = 2.0f; 

// OV3660 Camera FOV estimation (Tune these based on your specific resolution)
// If running at 640x480, focal length is roughly equivalent to these:
constexpr float FOCAL_LENGTH_X = 250.0f; 
constexpr float FOCAL_LENGTH_Y = 250.0f;
constexpr float IMAGE_CENTER_X = 160.0f; // width / 2
constexpr float IMAGE_CENTER_Y = 120.0f; // height / 2

// ─── DATA STRUCTURES ────────────────────────────────────────────────────
struct GroundPoint {
    float x; // Lateral distance (Left/Right) in meters
    float y; // Forward distance in meters
    bool valid; // True if the projection hits the floor, false if it hits the sky
};

// ─── FUNCTIONS ──────────────────────────────────────────────────────────
/**
 * @brief Projects a 2D camera pixel onto the 3D ground plane.
 * @param u The pixel X coordinate (0 to width)
 * @param v The pixel Y coordinate (0 to height)
 * @return GroundPoint containing the metric X, Y coordinates relative to the rover.
 */
GroundPoint pixelToGround(float u, float v);

} // namespace drift