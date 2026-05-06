#pragma once
#include "flow_accumulator.h"

namespace drift {

/**
 * @brief Initializes the OV3660 camera and allocates PSRAM buffers.
 * @return true on success, false on failure.
 */
bool cameraInit();

/**
 * @brief Captures a frame, runs FAST+LK, accumulates flow.
 * 
 * Backward-compatible interface for main.cpp.
 * Outputs are RAW PIXEL displacements, NOT meters.
 *
 * @param[out] dx         Lateral X displacement in pixels (+ = right)
 * @param[out] dy         Lateral Y displacement in pixels (+ = down)  
 * @param[out] confidence 0.0 - 1.0, how reliable this measurement is
 * @return true ONLY when a full 5-frame window has accumulated.
 *
 * Direction + radial data logged to serial and available via
 * cameraGetLastMeasurement().
 */
bool cameraProcessFrame(float &dx, float &dy, float &confidence);

/**
 * @brief Full measurement interface — returns CameraMeasurement struct.
 *
 * Contains: lateral_dx/dy, radial_total, direction, confidence, variance.
 * Use this when the EKF needs direction or radial data.
 */
bool cameraProcessFrame(CameraMeasurement &cam);

/**
 * @brief Get the last CameraMeasurement (after cameraProcessFrame returned true).
 */
const CameraMeasurement& cameraGetLastMeasurement();

/**
 * @brief Checks Serial for debug commands:
 *   't' = toggle manual test mode (step-by-step with frame saving)
 *   'r' = reset accumulator
 */
void cameraDebugCheck();

} // namespace drift