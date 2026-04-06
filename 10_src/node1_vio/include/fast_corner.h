#pragma once
#include <cstdint>

// ============================================================
// FAST-9 Corner Detector — PROJECT DRIFT
// Optimized for ESP32 / 320x240 grayscale frames
// ============================================================

struct Corner {
    uint16_t x;
    uint16_t y;
    int16_t  score;   // Corner response (used for NMS + sorting)
};

// Maximum corners we'll ever return (proposal targets 20-50)
static constexpr int MAX_CORNERS = 100;

// Detect FAST-9 corners in a grayscale image buffer.
//
//   img        : pointer to grayscale pixel data (row-major)
//   width      : image width  (320)
//   height     : image height (240)
//   threshold  : intensity diff threshold (start with 20-30, tune later)
//   out_corners: pre-allocated array of Corner structs
//
// Returns: number of corners found (capped at MAX_CORNERS)
int fast9_detect(const uint8_t *img, int width, int height,
                 int threshold, Corner *out_corners);

// Non-maximum suppression: suppress corners that have a neighbor
// with a higher score within `radius` pixels.
// Operates in-place on the corner array.
// Returns: new count after suppression.
int fast_nms(Corner *corners, int count, int radius);
