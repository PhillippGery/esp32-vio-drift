#pragma once
#include <cstdint>
#include <cmath>
#include "optical_flow.h"

// ============================================================
// Flow Accumulator — PROJECT DRIFT
//
// PURPOSE:
//   Accumulates optical flow over N frames and outputs:
//     1. Direction of motion (FORWARD, LEFT, RIGHT, etc.)
//     2. Raw pixel displacement (lateral + radial components)
//     3. Confidence score (0.0 - 1.0)
//
//   The camera DOES NOT output meters. Absolute distance
//   requires the IMU (accelerometer double-integration).
//   The EKF fuses camera direction with IMU scale.
//
// FLOW DECOMPOSITION:
//   Each flow vector is split into two components relative
//   to the image center (160, 120):
//
//   LATERAL = uniform component (all vectors same direction)
//             Caused by sideways / vertical camera motion
//
//   RADIAL  = expansion/contraction from image center
//             Caused by forward / backward camera motion
//
//   Moving sideways:     Moving forward:
//   ←←←←←←←←←←          ↖  ↑  ↗
//   ←←←←←←←←←←          ←  ·  →  (expanding)
//   ←←←←←←←←←←          ↙  ↓  ↘
//
// CONFIDENCE CALCULATION:
//   confidence = vector_score * consistency_score
//
//   vector_score = min(total_valid_vectors / 50, 1.0)
//     → More tracked features = more reliable measurement
//     → 50+ vectors = maximum score
//
//   consistency_score = 1.0 / (1.0 + variance * 2.0)
//     → Low variance between frames = consistent motion = reliable
//     → High variance = jerky/noisy motion = unreliable
//
//   Result: 0.0 = don't trust camera, 1.0 = fully trust camera
//   The EKF uses this to scale its measurement noise R matrix:
//     R_effective = R_base / confidence
// ============================================================

static constexpr int ACCUM_WINDOW = 5;

static constexpr float IMG_CX = 160.0f;  // Image center x (320/2)
static constexpr float IMG_CY = 120.0f;  // Image center y (240/2)

// Motion direction enum
enum MotionDirection {
    DIR_STATIONARY = 0,
    DIR_FORWARD    = 1,
    DIR_BACKWARD   = 2,
    DIR_LEFT       = 3,
    DIR_RIGHT      = 4,
    DIR_UP         = 5,
    DIR_DOWN       = 6
};

// Per-frame flow decomposition
struct FlowDecomposition {
    float lateral_dx;   // Mean uniform horizontal flow (pixels)
    float lateral_dy;   // Mean uniform vertical flow (pixels)
    float radial_mean;  // Mean radial expansion (+ = forward, - = backward)
    int   n_valid;      // Number of valid flow vectors this frame
};

// Camera measurement — what the EKF receives
struct CameraMeasurement {
    // Raw pixel displacements (accumulated over N frames)
    float lateral_dx;       // Total lateral x displacement (pixels)
    float lateral_dy;       // Total lateral y displacement (pixels)
    float radial_total;     // Total radial expansion (pixels, + = forward)

    // Direction
    MotionDirection direction;  // Dominant motion direction
    float direction_angle;      // Angle in degrees (0=right, 90=down, 180=left)

    // Confidence (0.0 - 1.0)
    float confidence;

    // Variance (for EKF R matrix tuning)
    float variance_lateral;     // Variance of per-frame lateral magnitudes
    float variance_radial;      // Variance of per-frame radial values

    // Metadata
    int   n_frames;         // Number of frames accumulated
    int   total_vectors;    // Total valid flow vectors used
    int64_t timestamp;      // Microsecond timestamp

    bool valid;             // True if measurement is usable
};

class FlowAccumulator {
public:
    FlowAccumulator() { reset(); }

    void reset() {
        m_frame_count = 0;
        m_total_vectors = 0;
        m_sum_dx = 0; m_sum_dy = 0; m_sum_radial = 0;
        m_sum_dx2 = 0; m_sum_dy2 = 0; m_sum_radial2 = 0;
    }

    // Decompose one frame's flow into lateral + radial
    static FlowDecomposition decompose_frame(const FlowVector *flow, int n_flow) {
        FlowDecomposition d = {};
        float sum_lat_dx = 0, sum_lat_dy = 0, sum_radial = 0;
        int n_valid = 0;

        for (int i = 0; i < n_flow; i++) {
            if (!flow[i].valid) continue;

            // Vector from image center to corner position
            float rx = flow[i].px - IMG_CX;
            float ry = flow[i].py - IMG_CY;
            float r_dist = sqrtf(rx * rx + ry * ry);

            if (r_dist < 5.0f) {
                // Too close to center — treat as pure lateral
                sum_lat_dx += flow[i].dx;
                sum_lat_dy += flow[i].dy;
                n_valid++;
                continue;
            }

            // Radial unit vector (outward from center)
            float ur_x = rx / r_dist;
            float ur_y = ry / r_dist;

            // Radial component = dot(flow, radial_unit)
            float radial = flow[i].dx * ur_x + flow[i].dy * ur_y;

            // Lateral component = flow minus radial
            float lat_x = flow[i].dx - radial * ur_x;
            float lat_y = flow[i].dy - radial * ur_y;

            // Normalize radial by distance from center
            float radial_norm = radial / (r_dist / 100.0f);

            sum_lat_dx += lat_x;
            sum_lat_dy += lat_y;
            sum_radial += radial_norm;
            n_valid++;
        }

        if (n_valid > 0) {
            d.lateral_dx = sum_lat_dx / (float)n_valid;
            d.lateral_dy = sum_lat_dy / (float)n_valid;
            d.radial_mean = sum_radial / (float)n_valid;
        }
        d.n_valid = n_valid;
        return d;
    }

    // Add one frame's flow vectors
    bool add_frame(const FlowVector *flow, int n_flow,
                   FlowDecomposition *out_decomp = nullptr) {
        FlowDecomposition d = decompose_frame(flow, n_flow);
        if (out_decomp) *out_decomp = d;

        if (d.n_valid == 0) {
            m_frame_count++;
            return false;
        }

        m_sum_dx += d.lateral_dx;
        m_sum_dy += d.lateral_dy;
        m_sum_radial += d.radial_mean;

        m_sum_dx2 += d.lateral_dx * d.lateral_dx;
        m_sum_dy2 += d.lateral_dy * d.lateral_dy;
        m_sum_radial2 += d.radial_mean * d.radial_mean;

        m_total_vectors += d.n_valid;
        m_frame_count++;
        return true;
    }

    bool is_ready() const { return m_frame_count >= ACCUM_WINDOW; }
    int frame_count() const { return m_frame_count; }

    void get_pixel_totals(float &dx, float &dy, float &radial) const {
        dx = m_sum_dx;
        dy = m_sum_dy;
        radial = m_sum_radial;
    }

    // Determine dominant direction
    static MotionDirection classify_direction(float lat_dx, float lat_dy, float radial) {
        float lat_mag = sqrtf(lat_dx * lat_dx + lat_dy * lat_dy);
        float rad_mag = fabsf(radial);

        if (lat_mag < 1.0f && rad_mag < 1.0f) return DIR_STATIONARY;

        if (rad_mag > lat_mag && rad_mag > 1.0f) {
            return radial > 0 ? DIR_FORWARD : DIR_BACKWARD;
        }

        float angle = atan2f(lat_dy, lat_dx) * 180.0f / 3.14159f;
        if (angle > -45 && angle <= 45) return DIR_RIGHT;
        if (angle > 45 && angle <= 135) return DIR_DOWN;
        if (angle > 135 || angle <= -135) return DIR_LEFT;
        return DIR_UP;
    }

    static const char* direction_name(MotionDirection dir) {
        switch (dir) {
            case DIR_STATIONARY: return "STATIONARY";
            case DIR_FORWARD:    return "FORWARD";
            case DIR_BACKWARD:   return "BACKWARD";
            case DIR_LEFT:       return "LEFT";
            case DIR_RIGHT:      return "RIGHT";
            case DIR_UP:         return "UP";
            case DIR_DOWN:       return "DOWN";
            default:             return "UNKNOWN";
        }
    }

    // Produce final camera measurement for the EKF
    CameraMeasurement get_measurement(int64_t timestamp) const {
        CameraMeasurement m = {};
        m.timestamp = timestamp;
        m.n_frames = m_frame_count;
        m.total_vectors = m_total_vectors;

        if (m_frame_count < ACCUM_WINDOW || m_total_vectors < 3) {
            m.valid = false;
            return m;
        }

        // Pixel totals
        m.lateral_dx = m_sum_dx;
        m.lateral_dy = m_sum_dy;
        m.radial_total = m_sum_radial;

        // Direction
        m.direction = classify_direction(m_sum_dx, m_sum_dy, m_sum_radial);
        m.direction_angle = atan2f(m_sum_dy, m_sum_dx) * 180.0f / 3.14159f;

        // Variance
        float n = (float)m_frame_count;
        float mdx = m_sum_dx / n, mdy = m_sum_dy / n, mrad = m_sum_radial / n;
        m.variance_lateral = (m_sum_dx2 / n) - (mdx * mdx) +
                             (m_sum_dy2 / n) - (mdy * mdy);
        m.variance_radial = (m_sum_radial2 / n) - (mrad * mrad);
        if (m.variance_lateral < 0) m.variance_lateral = 0;
        if (m.variance_radial < 0) m.variance_radial = 0;

        // Confidence = vector_score * consistency_score
        float vec_score = fminf((float)m_total_vectors / 50.0f, 1.0f);
        float var_total = sqrtf(m.variance_lateral + m.variance_radial);
        float var_score = 1.0f / (1.0f + var_total * 2.0f);
        m.confidence = vec_score * var_score;

        m.valid = true;
        return m;
    }

private:
    int   m_frame_count;
    int   m_total_vectors;
    float m_sum_dx, m_sum_dy, m_sum_radial;
    float m_sum_dx2, m_sum_dy2, m_sum_radial2;
};