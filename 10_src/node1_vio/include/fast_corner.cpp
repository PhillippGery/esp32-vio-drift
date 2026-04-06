// ============================================================
// FAST-9 Corner Detector — PROJECT DRIFT
//
// References:
//   Rosten, Porter & Drummond, "Faster and Better: A Machine
//   Learning Approach to Corner Detection," IEEE TPAMI 2010.
//
// Implementation notes for ESP32:
//   - No heap allocation — caller provides output array
//   - 3-pixel border margin to keep circle in bounds
//   - High-speed reject: check 4 cardinal pixels first
//   - Score = sum of absolute diffs for NMS ranking
// ============================================================

#include "fast_corner.h"
#include <cstdlib>  // abs()

// 16-pixel Bresenham circle offsets at radius 3
// Indexed 0-15 going clockwise from top
// Each entry is {dx, dy} relative to center pixel
static const int8_t circle[16][2] = {
    { 0, -3}, { 1, -3}, { 2, -2}, { 3, -1},   //  0- 3: top to right
    { 3,  0}, { 3,  1}, { 2,  2}, { 1,  3},   //  4- 7: right to bottom
    { 0,  3}, {-1,  3}, {-2,  2}, {-3,  1},   //  8-11: bottom to left
    {-3,  0}, {-3, -1}, {-2, -2}, {-1, -3}    // 12-15: left to top
};

// Compute corner score = sum of |pixel - center| for all 16 circle pixels
// Higher score = stronger corner = survives NMS
static int corner_score(const uint8_t *img, int stride, int cx, int cy, int threshold) {
    int center = img[cy * stride + cx];
    int score = 0;
    for (int i = 0; i < 16; i++) {
        int px = img[(cy + circle[i][1]) * stride + (cx + circle[i][0])];
        int diff = abs(px - center);
        if (diff > threshold) {
            score += diff - threshold;
        }
    }
    return score;
}

int fast9_detect(const uint8_t *img, int width, int height,
                 int threshold, Corner *out_corners) {
    int count = 0;
    const int BORDER = 3;  // Bresenham circle radius

    for (int y = BORDER; y < height - BORDER; y++) {
        for (int x = BORDER; x < width - BORDER; x++) {
            if (count >= MAX_CORNERS) goto done;

            int center = img[y * width + x];
            int ct = center + threshold;  // brighter threshold
            int c_t = center - threshold; // darker threshold

            // ------------------------------------------------
            // High-speed test: check pixels 0, 4, 8, 12
            // (top, right, bottom, left on the circle)
            // At least 3 of 4 must be ALL brighter or ALL darker
            // Otherwise it can't be a FAST-9 corner — skip
            // ------------------------------------------------
            int p0  = img[(y + circle[0][1])  * width + (x + circle[0][0])];
            int p4  = img[(y + circle[4][1])  * width + (x + circle[4][0])];
            int p8  = img[(y + circle[8][1])  * width + (x + circle[8][0])];
            int p12 = img[(y + circle[12][1]) * width + (x + circle[12][0])];

            // Count how many cardinal pixels are brighter / darker
            int n_bright = (p0 > ct) + (p4 > ct) + (p8 > ct) + (p12 > ct);
            int n_dark   = (p0 < c_t) + (p4 < c_t) + (p8 < c_t) + (p12 < c_t);

            if (n_bright < 3 && n_dark < 3) continue;  // Quick reject

            // ------------------------------------------------
            // Full test: check for 9 contiguous pixels on the
            // 16-pixel circle that are ALL brighter or ALL darker
            // ------------------------------------------------
            // Load all 16 circle pixels
            int px[16];
            px[0] = p0; px[4] = p4; px[8] = p8; px[12] = p12;
            for (int i = 0; i < 16; i++) {
                if (i == 0 || i == 4 || i == 8 || i == 12) continue;
                px[i] = img[(y + circle[i][1]) * width + (x + circle[i][0])];
            }

            // Check for 9 contiguous brighter
            bool is_corner = false;

            if (n_bright >= 3) {
                // Try brighter arc
                int run = 0;
                for (int k = 0; k < 25; k++) {  // 16 + 9 - 1 to handle wrap
                    if (px[k % 16] > ct) {
                        run++;
                        if (run >= 9) { is_corner = true; break; }
                    } else {
                        run = 0;
                    }
                }
            }

            if (!is_corner && n_dark >= 3) {
                // Try darker arc
                int run = 0;
                for (int k = 0; k < 25; k++) {
                    if (px[k % 16] < c_t) {
                        run++;
                        if (run >= 9) { is_corner = true; break; }
                    } else {
                        run = 0;
                    }
                }
            }

            if (is_corner) {
                out_corners[count].x = x;
                out_corners[count].y = y;
                out_corners[count].score = corner_score(img, width, x, y, threshold);
                count++;
            }
        }
    }
done:
    return count;
}

int fast_nms(Corner *corners, int count, int radius) {
    // Simple suppression: mark weaker corners near stronger ones
    // Using a suppress flag array on the stack (MAX_CORNERS is small)
    bool suppress[MAX_CORNERS] = {false};
    int r2 = radius * radius;

    for (int i = 0; i < count; i++) {
        if (suppress[i]) continue;
        for (int j = i + 1; j < count; j++) {
            if (suppress[j]) continue;
            int dx = corners[i].x - corners[j].x;
            int dy = corners[i].y - corners[j].y;
            if (dx * dx + dy * dy <= r2) {
                // Suppress the weaker one
                if (corners[i].score >= corners[j].score) {
                    suppress[j] = true;
                } else {
                    suppress[i] = true;
                    break;  // i is suppressed, move on
                }
            }
        }
    }

    // Compact the array
    int out = 0;
    for (int i = 0; i < count; i++) {
        if (!suppress[i]) {
            corners[out++] = corners[i];
        }
    }
    return out;
}
