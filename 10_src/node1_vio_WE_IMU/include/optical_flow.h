#pragma once
#include <cstdint>
#include "fast_corner.h"

// ============================================================
// Lucas-Kanade Sparse Optical Flow — PROJECT DRIFT
// Optimized for ESP32 / 320x240 grayscale frames
// ============================================================

struct FlowVector {
    float px;           // Corner position in previous frame
    float py;
    float dx;           // Displacement: new_x = px + dx
    float dy;           // Displacement: new_y = py + dy
    bool  valid;        // true if tracking succeeded
};

// Configuration
static constexpr int LK_WINDOW    = 7;
static constexpr int LK_HALF_WIN  = 3;
static constexpr int LK_MAX_ITER  = 5;
static constexpr float LK_EPSILON = 0.01f;
static constexpr float LK_MIN_DET = 1.0f;

static constexpr int MAX_FLOW = MAX_CORNERS;

// Run Lucas-Kanade optical flow.
// Returns: number of successfully tracked features
int lk_optical_flow(const uint8_t *prev_img, const uint8_t *curr_img,
                    int width, int height,
                    const Corner *corners, int n_corners,
                    FlowVector *out_flow);
