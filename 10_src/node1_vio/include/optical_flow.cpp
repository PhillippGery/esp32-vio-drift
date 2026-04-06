// ============================================================
// Lucas-Kanade Sparse Optical Flow — PROJECT DRIFT
//
// References:
//   B. Lucas and T. Kanade, "An Iterative Image Registration
//   Technique with an Application to Stereo Vision," IJCAI 1981.
//
// Implementation notes for ESP32:
//   - No heap allocation — all buffers on stack or caller-provided
//   - Bilinear interpolation for sub-pixel accuracy
//   - Central differences for gradients
//   - 7x7 window, 5 iterations max
//   - Features rejected if: out of bounds, low texture, or divergent
// ============================================================

#include "optical_flow.h"
#include <cmath>

// Bilinear interpolation at sub-pixel position
static inline float bilinear_sample(const uint8_t *img, int width, int height,
                                    float x, float y) {
    int ix = (int)x;
    int iy = (int)y;
    if (ix < 0 || iy < 0 || ix >= width - 1 || iy >= height - 1)
        return -1.0f;

    float fx = x - ix;
    float fy = y - iy;
    const uint8_t *row0 = img + iy * width + ix;
    const uint8_t *row1 = row0 + width;

    return (1.0f - fx) * (1.0f - fy) * row0[0]
         + fx         * (1.0f - fy) * row0[1]
         + (1.0f - fx) * fy         * row1[0]
         + fx         * fy          * row1[1];
}

// Track a single corner from prev_img to curr_img
static bool track_single(const uint8_t *prev_img, const uint8_t *curr_img,
                          int width, int height,
                          float px, float py,
                          float *out_dx, float *out_dy) {
    int ipx = (int)(px + 0.5f);
    int ipy = (int)(py + 0.5f);
    if (ipx - LK_HALF_WIN < 1 || ipx + LK_HALF_WIN >= width - 1 ||
        ipy - LK_HALF_WIN < 1 || ipy + LK_HALF_WIN >= height - 1)
        return false;

    // Precompute gradients and structure tensor
    float Gxx = 0, Gxy = 0, Gyy = 0;
    float Ix_win[LK_WINDOW * LK_WINDOW];
    float Iy_win[LK_WINDOW * LK_WINDOW];

    int idx = 0;
    for (int wy = -LK_HALF_WIN; wy <= LK_HALF_WIN; wy++) {
        for (int wx = -LK_HALF_WIN; wx <= LK_HALF_WIN; wx++) {
            int sx = ipx + wx;
            int sy = ipy + wy;
            float ix = (float)(prev_img[sy * width + sx + 1] -
                               prev_img[sy * width + sx - 1]) * 0.5f;
            float iy = (float)(prev_img[(sy + 1) * width + sx] -
                               prev_img[(sy - 1) * width + sx]) * 0.5f;
            Ix_win[idx] = ix;
            Iy_win[idx] = iy;
            idx++;
            Gxx += ix * ix;
            Gxy += ix * iy;
            Gyy += iy * iy;
        }
    }

    float det = Gxx * Gyy - Gxy * Gxy;
    if (fabsf(det) < LK_MIN_DET) return false;

    float inv_det = 1.0f / det;
    float Ginv00 =  Gyy * inv_det;
    float Ginv01 = -Gxy * inv_det;
    float Ginv11 =  Gxx * inv_det;

    float dx = 0.0f, dy = 0.0f;

    for (int iter = 0; iter < LK_MAX_ITER; iter++) {
        float bx = 0.0f, by = 0.0f;
        idx = 0;
        bool oob = false;

        for (int wy = -LK_HALF_WIN; wy <= LK_HALF_WIN; wy++) {
            for (int wx = -LK_HALF_WIN; wx <= LK_HALF_WIN; wx++) {
                float prev_val = (float)prev_img[(ipy + wy) * width + (ipx + wx)];
                float curr_val = bilinear_sample(curr_img, width, height,
                                                 px + wx + dx, py + wy + dy);
                if (curr_val < 0) { oob = true; break; }

                float it = curr_val - prev_val;
                bx += it * Ix_win[idx];
                by += it * Iy_win[idx];
                idx++;
            }
            if (oob) break;
        }
        if (oob) return false;

        float ddx = -(Ginv00 * bx + Ginv01 * by);
        float ddy = -(Ginv01 * bx + Ginv11 * by);
        dx += ddx;
        dy += ddy;

        if (fabsf(ddx) < LK_EPSILON && fabsf(ddy) < LK_EPSILON) break;
        if (fabsf(dx) > LK_WINDOW * 2 || fabsf(dy) > LK_WINDOW * 2) return false;
    }

    float new_x = px + dx;
    float new_y = py + dy;
    if (new_x < 0 || new_x >= width - 1 || new_y < 0 || new_y >= height - 1)
        return false;

    *out_dx = dx;
    *out_dy = dy;
    return true;
}

// Run LK optical flow on all corners
int lk_optical_flow(const uint8_t *prev_img, const uint8_t *curr_img,
                    int width, int height,
                    const Corner *corners, int n_corners,
                    FlowVector *out_flow) {
    int n_tracked = 0;
    int n = (n_corners > MAX_FLOW) ? MAX_FLOW : n_corners;

    for (int i = 0; i < n; i++) {
        float px = (float)corners[i].x;
        float py = (float)corners[i].y;
        float dx = 0, dy = 0;

        bool ok = track_single(prev_img, curr_img, width, height,
                                px, py, &dx, &dy);

        out_flow[i].px    = px;
        out_flow[i].py    = py;
        out_flow[i].dx    = dx;
        out_flow[i].dy    = dy;
        out_flow[i].valid = ok;

        if (ok) n_tracked++;
    }

    return n_tracked;
}
