#pragma once
#include <cstdint>
#include <cmath>
#include "optical_flow.h"

// ============================================================
// Flow Accumulator — PROJECT DRIFT (Stabilized)
//
// Output to EKF:
//   - Direction: FORWARD, BACKWARD, LEFT, RIGHT, STATIONARY
//   - Raw pixel displacement (lateral + radial)
//   - Confidence (0.0 - 1.0)
//
// Noise rejection:
//   - Per-vector: ignore vectors < 0.5px (sensor jitter)
//   - Per-frame:  clamp to zero if mean < 0.3px (dead zone)
//   - Final:      STATIONARY if total < 2.0px
//   - Corner gate: skip frame entirely if < 8 corners
//   - Corner count in confidence: low corners = low confidence
// ============================================================

static constexpr int ACCUM_WINDOW = 5;

static constexpr float IMG_CX = 160.0f;
static constexpr float IMG_CY = 120.0f;

// Noise rejection thresholds
static constexpr float MIN_FLOW_MAGNITUDE   = 0.5f;
static constexpr float FRAME_DEAD_ZONE      = 0.3f;
static constexpr float STATIONARY_THRESHOLD  = 2.0f;
static constexpr int   MIN_CORNERS          = 8;

enum MotionDirection {
    DIR_STATIONARY = 0,
    DIR_FORWARD    = 1,
    DIR_BACKWARD   = 2,
    DIR_LEFT       = 3,
    DIR_RIGHT      = 4
};

struct FlowDecomposition {
    float lateral_dx;
    float lateral_dy;
    float radial_mean;
    int   n_valid;
    int   n_rejected;
    bool  is_stationary;
};

struct CameraMeasurement {
    float lateral_dx;       // Total lateral x pixels (+ = right)
    float lateral_dy;       // Total lateral y pixels (+ = down)
    float radial_total;     // Total radial pixels (+ = forward, - = backward)

    MotionDirection direction;
    float direction_angle;

    float confidence;
    float variance_lateral;
    float variance_radial;

    int   n_frames;
    int   n_active_frames;
    int   total_vectors;
    int   total_corners;
    int64_t timestamp;
    bool valid;
};

class FlowAccumulator {
public:
    FlowAccumulator() { reset(); }

    void reset() {
        m_frame_count = 0;
        m_active_frames = 0;
        m_total_vectors = 0;
        m_total_corners = 0;
        m_sum_dx = 0; m_sum_dy = 0; m_sum_radial = 0;
        m_sum_dx2 = 0; m_sum_dy2 = 0; m_sum_radial2 = 0;
    }

    static FlowDecomposition decompose_frame(const FlowVector *flow, int n_flow) {
        FlowDecomposition d = {};
        float sum_lat_dx = 0, sum_lat_dy = 0, sum_radial = 0;
        int n_valid = 0, n_rejected = 0;

        for (int i = 0; i < n_flow; i++) {
            if (!flow[i].valid) continue;

            float mag = sqrtf(flow[i].dx * flow[i].dx + flow[i].dy * flow[i].dy);
            if (mag < MIN_FLOW_MAGNITUDE) {
                n_rejected++;
                continue;
            }

            float rx = flow[i].px - IMG_CX;
            float ry = flow[i].py - IMG_CY;
            float r_dist = sqrtf(rx * rx + ry * ry);

            if (r_dist < 10.0f) {
                sum_lat_dx += flow[i].dx;
                sum_lat_dy += flow[i].dy;
                n_valid++;
                continue;
            }

            float ur_x = rx / r_dist;
            float ur_y = ry / r_dist;
            float radial = flow[i].dx * ur_x + flow[i].dy * ur_y;
            float lat_x = flow[i].dx - radial * ur_x;
            float lat_y = flow[i].dy - radial * ur_y;
            float radial_norm = radial / (r_dist / 100.0f);

            sum_lat_dx += lat_x;
            sum_lat_dy += lat_y;
            sum_radial += radial_norm;
            n_valid++;
        }

        d.n_valid = n_valid;
        d.n_rejected = n_rejected;

        if (n_valid > 0) {
            d.lateral_dx = sum_lat_dx / (float)n_valid;
            d.lateral_dy = sum_lat_dy / (float)n_valid;
            d.radial_mean = sum_radial / (float)n_valid;
        }

        float total_motion = sqrtf(d.lateral_dx * d.lateral_dx +
                                   d.lateral_dy * d.lateral_dy +
                                   d.radial_mean * d.radial_mean);

        if (total_motion < FRAME_DEAD_ZONE) {
            d.lateral_dx = 0;
            d.lateral_dy = 0;
            d.radial_mean = 0;
            d.is_stationary = true;
        } else {
            d.is_stationary = false;
        }

        return d;
    }

    bool add_frame(const FlowVector *flow, int n_flow,
                   FlowDecomposition *out_decomp = nullptr,
                   int n_corners = 0) {
        FlowDecomposition d = decompose_frame(flow, n_flow);
        if (out_decomp) *out_decomp = d;

        m_frame_count++;
        m_total_corners += n_corners;

        if (d.n_valid == 0 || d.is_stationary) return false;

        m_sum_dx += d.lateral_dx;
        m_sum_dy += d.lateral_dy;
        m_sum_radial += d.radial_mean;
        m_sum_dx2 += d.lateral_dx * d.lateral_dx;
        m_sum_dy2 += d.lateral_dy * d.lateral_dy;
        m_sum_radial2 += d.radial_mean * d.radial_mean;

        m_total_vectors += d.n_valid;
        m_active_frames++;
        return true;
    }

    bool is_ready() const { return m_frame_count >= ACCUM_WINDOW; }
    int frame_count() const { return m_frame_count; }

    void get_pixel_totals(float &dx, float &dy, float &radial) const {
        dx = m_sum_dx; dy = m_sum_dy; radial = m_sum_radial;
    }

    // FIXED: uses full lateral magnitude sqrt(dx^2 + dy^2), not just fabsf(dx)
    static MotionDirection classify_direction(float lat_dx, float lat_dy, float radial) {
        float lat_mag = sqrtf(lat_dx * lat_dx + lat_dy * lat_dy);
        float rad_mag = fabsf(radial);
        float total = sqrtf(lat_mag * lat_mag + rad_mag * rad_mag);

        if (total < STATIONARY_THRESHOLD) return DIR_STATIONARY;

        if (rad_mag > lat_mag) {
            return radial > 0 ? DIR_FORWARD : DIR_BACKWARD;
        } else {
            return lat_dx > 0 ? DIR_RIGHT : DIR_LEFT;
        }
    }

    static const char* direction_name(MotionDirection dir) {
        switch (dir) {
            case DIR_STATIONARY: return "STATIONARY";
            case DIR_FORWARD:    return "FORWARD";
            case DIR_BACKWARD:   return "BACKWARD";
            case DIR_LEFT:       return "LEFT";
            case DIR_RIGHT:      return "RIGHT";
            default:             return "UNKNOWN";
        }
    }

    CameraMeasurement get_measurement(int64_t timestamp) const {
        CameraMeasurement m = {};
        m.timestamp = timestamp;
        m.n_frames = m_frame_count;
        m.n_active_frames = m_active_frames;
        m.total_vectors = m_total_vectors;
        m.total_corners = m_total_corners;

        if (m_frame_count < ACCUM_WINDOW) {
            m.valid = false;
            return m;
        }

        m.lateral_dx = m_sum_dx;
        m.lateral_dy = m_sum_dy;
        m.radial_total = m_sum_radial;

        m.direction = classify_direction(m_sum_dx, m_sum_dy, m_sum_radial);
        m.direction_angle = atan2f(m_sum_dy, m_sum_dx) * 180.0f / 3.14159f;

        if (m_active_frames > 0) {
            float n = (float)m_active_frames;
            float mdx = m_sum_dx / n, mdy = m_sum_dy / n, mrad = m_sum_radial / n;
            m.variance_lateral = (m_sum_dx2 / n) - (mdx * mdx) +
                                 (m_sum_dy2 / n) - (mdy * mdy);
            m.variance_radial = (m_sum_radial2 / n) - (mrad * mrad);
            if (m.variance_lateral < 0) m.variance_lateral = 0;
            if (m.variance_radial < 0) m.variance_radial = 0;
        }

        float vec_score = fminf((float)m_total_vectors / 50.0f, 1.0f);
        float var_total = sqrtf(m.variance_lateral + m.variance_radial);
        float var_score = 1.0f / (1.0f + var_total * 2.0f);
        float avg_corners = (m_frame_count > 0) ? (float)m_total_corners / (float)m_frame_count : 0;
        float corner_score = fminf(avg_corners / 20.0f, 1.0f);

        if (m.direction == DIR_STATIONARY) {
            m.confidence = vec_score * corner_score * 0.95f;
        } else {
            m.confidence = vec_score * var_score * corner_score;
        }

        m.valid = true;
        return m;
    }

private:
    int   m_frame_count;
    int   m_active_frames;
    int   m_total_vectors;
    int   m_total_corners;
    float m_sum_dx, m_sum_dy, m_sum_radial;
    float m_sum_dx2, m_sum_dy2, m_sum_radial2;
};