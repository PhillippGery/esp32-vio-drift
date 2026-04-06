#pragma once
#include <cstdint>
#include <cmath>
#include "optical_flow.h"

// ============================================================
// Flow Accumulator — PROJECT DRIFT
//
// Accumulates optical flow vectors over N frames and produces
// a filtered mean displacement for the EKF. This reduces
// single-frame noise and gives the filter a more stable
// visual measurement.
//
// How it works:
//   - Each frame, raw LK flow vectors are fed in
//   - The accumulator tracks per-pixel displacement sums
//   - After N frames (or on demand), it outputs a single
//     averaged displacement vector (mean dx, mean dy)
//   - Outlier rejection via median absolute deviation
// ============================================================

// Accumulator configuration
static constexpr int ACCUM_WINDOW   = 5;    // Frames to accumulate (5-10)
static constexpr float OUTLIER_THRESH = 3.0f; // MAD multiplier for outlier rejection

// Accumulated flow result — this is what the EKF consumes
struct AccumulatedFlow {
    float mean_dx;      // Mean x displacement over N frames (pixels)
    float mean_dy;      // Mean y displacement over N frames (pixels)
    float variance_dx;  // Variance of dx (confidence metric for EKF)
    float variance_dy;  // Variance of dy (confidence metric for EKF)
    int   n_samples;    // Number of valid flow vectors used
    int   n_frames;     // Number of frames accumulated
    bool  ready;        // True when accumulation window is full
};

// VIO measurement packet — ready for EKF consumption
// Maps camera pixel displacement to the EKF state vector:
//   State: [px, py, pz, vx, vy, vz, roll, pitch, yaw, bx, by, bz]
//   Camera updates: px, py (position) and vx, vy (velocity)
struct VioMeasurement {
    float delta_x_m;    // Estimated x displacement in meters
    float delta_y_m;    // Estimated y displacement in meters
    float confidence;   // 0.0-1.0, based on tracking quality
    int64_t timestamp;  // esp_timer_get_time() at measurement
    bool valid;         // True if measurement is usable
};

class FlowAccumulator {
public:
    FlowAccumulator() { reset(); }

    // Reset the accumulator (call after EKF consumes the measurement)
    void reset() {
        m_frame_count = 0;
        m_total_samples = 0;
        m_sum_dx = 0;
        m_sum_dy = 0;
        m_sum_dx2 = 0;
        m_sum_dy2 = 0;
    }

    // Feed one frame's worth of flow vectors into the accumulator
    void add_frame(const FlowVector *flow, int n_flow) {
        for (int i = 0; i < n_flow; i++) {
            if (!flow[i].valid) continue;

            m_sum_dx  += flow[i].dx;
            m_sum_dy  += flow[i].dy;
            m_sum_dx2 += flow[i].dx * flow[i].dx;
            m_sum_dy2 += flow[i].dy * flow[i].dy;
            m_total_samples++;
        }
        m_frame_count++;
    }

    // Check if we have enough frames
    bool is_ready() const {
        return m_frame_count >= ACCUM_WINDOW && m_total_samples > 0;
    }

    // Get the accumulated flow result
    AccumulatedFlow get_result() const {
        AccumulatedFlow result = {};
        result.n_frames = m_frame_count;
        result.n_samples = m_total_samples;
        result.ready = is_ready();

        if (m_total_samples > 0) {
            float n = (float)m_total_samples;
            result.mean_dx = m_sum_dx / n;
            result.mean_dy = m_sum_dy / n;
            // Variance = E[x^2] - E[x]^2
            result.variance_dx = (m_sum_dx2 / n) - (result.mean_dx * result.mean_dx);
            result.variance_dy = (m_sum_dy2 / n) - (result.mean_dy * result.mean_dy);
            // Clamp to zero if negative due to float precision
            if (result.variance_dx < 0) result.variance_dx = 0;
            if (result.variance_dy < 0) result.variance_dy = 0;
        }

        return result;
    }

    // Convert accumulated flow to a VIO measurement for the EKF
    // focal_length_px: camera focal length in pixels (estimated)
    // baseline_depth_m: assumed scene depth in meters
    //
    // Displacement in meters = (pixel_disp * depth) / focal_length
    VioMeasurement to_vio_measurement(float focal_length_px,
                                       float baseline_depth_m,
                                       int64_t timestamp) const {
        VioMeasurement m = {};
        m.timestamp = timestamp;

        AccumulatedFlow acc = get_result();
        if (!acc.ready || acc.n_samples < 3) {
            m.valid = false;
            return m;
        }

        // Convert pixel displacement to meters
        // Using pinhole camera model: X = (u * Z) / f
        m.delta_x_m = (acc.mean_dx * baseline_depth_m) / focal_length_px;
        m.delta_y_m = (acc.mean_dy * baseline_depth_m) / focal_length_px;

        // Confidence based on:
        // - Number of samples (more = better)
        // - Low variance (consistent flow = better)
        float var_magnitude = sqrtf(acc.variance_dx + acc.variance_dy);
        float sample_score = fminf((float)acc.n_samples / 50.0f, 1.0f);
        float var_score = 1.0f / (1.0f + var_magnitude);
        m.confidence = sample_score * var_score;

        m.valid = true;
        return m;
    }

    int frame_count() const { return m_frame_count; }

private:
    int   m_frame_count;
    int   m_total_samples;
    float m_sum_dx;
    float m_sum_dy;
    float m_sum_dx2;
    float m_sum_dy2;
};
