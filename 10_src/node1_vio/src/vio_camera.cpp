//panchitos space
//
// STABILIZED VERSION — All fixes included:
//   - Corner gate: skip frame if < 8 corners (Vedant's dark room fix)
//   - Per-vector noise filter: ignores sub-pixel jitter (< 0.5px)
//   - Per-frame dead zone: clamps tiny motion to zero (< 0.3px)
//   - Final threshold: STATIONARY unless total > 2.0px
//   - Corner count in confidence: low corners = low confidence
//   - 4 directions only: FORWARD, BACKWARD, LEFT, RIGHT, STATIONARY
//   - FIXED: classify_direction uses full lateral magnitude
//   - FIXED: consistent output mapping (dx=lateral_x, dy=lateral_y)
//
// SERIAL COMMANDS:
//   't' = Manual test mode (step-by-step with frame saving)
//   'r' = Reset accumulator

#include "vio_camera.h"
#include <Arduino.h>
#include <cstring>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "camera_pins.h"
#include "fast_corner.h"
#include "optical_flow.h"
#include "flow_accumulator.h"

namespace drift {

static const char *TAG = "CAM_VIO";

// ─── Constants ───────────────────────────────────────────────────────────
static constexpr int FAST_THRESHOLD = 20;
static constexpr int NMS_RADIUS     = 8;
static constexpr int FRAME_W = 320;
static constexpr int FRAME_H = 240;
static constexpr int FRAME_BYTES = FRAME_W * FRAME_H;

// ─── State ───────────────────────────────────────────────────────────────
static uint8_t *prev_frame = nullptr;
static Corner   prev_corners[MAX_CORNERS];
static int      prev_n_corners = 0;
static bool     has_prev_frame = false;

static FlowAccumulator flow_accum;

// Store last measurement for the full-struct accessor
static CameraMeasurement last_measurement = {};

static bool test_mode = false;
static int  test_frame_num = 0;

static float test_lat_dx[ACCUM_WINDOW];
static float test_lat_dy[ACCUM_WINDOW];
static float test_radial[ACCUM_WINDOW];
static int   test_vectors[ACCUM_WINDOW];
static bool  test_stationary[ACCUM_WINDOW];
static int   test_corners[ACCUM_WINDOW];

// Sync markers for test_viewer.py
static const uint8_t TEST_SYNC[]   = {0xDE, 0xAD, 0xBE, 0xEF};
static const uint8_t FRAME_SYNC[]  = {0xFF, 0xD8, 0xBE, 0xEF};
static const uint8_t CORNER_SYNC[] = {0xC0, 0x52, 0x4E, 0x52};
static const uint8_t FLOW_SYNC[]   = {0xF1, 0x0E, 0xDA, 0x7A};

// ─── Internal: capture frame ─────────────────────────────────────────────
static camera_fb_t* capture_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) ESP_LOGE(TAG, "Capture failed");
    return fb;
}

// ─── Internal: send frame data for test_viewer.py ────────────────────────
static void send_frame_data(camera_fb_t *fb,
                            Corner *corners, int n_corners,
                            FlowVector *flow, int n_flow,
                            int frame_num, FlowDecomposition *decomp) {
    vTaskDelay(pdMS_TO_TICKS(30));

    Serial.write(TEST_SYNC, 4);
    uint8_t fnum = (uint8_t)frame_num;
    Serial.write(&fnum, 1);
    if (decomp) {
        Serial.write((uint8_t *)&decomp->lateral_dx, 4);
        Serial.write((uint8_t *)&decomp->lateral_dy, 4);
        Serial.write((uint8_t *)&decomp->radial_mean, 4);
    } else {
        float zero = 0;
        Serial.write((uint8_t *)&zero, 4);
        Serial.write((uint8_t *)&zero, 4);
        Serial.write((uint8_t *)&zero, 4);
    }

    Serial.write(FRAME_SYNC, 4);
    Serial.write(fb->buf, fb->len);

    Serial.write(CORNER_SYNC, 4);
    uint16_t cc = (uint16_t)n_corners;
    Serial.write((uint8_t *)&cc, 2);
    for (int i = 0; i < n_corners; i++) {
        Serial.write((uint8_t *)&corners[i].x, 2);
        Serial.write((uint8_t *)&corners[i].y, 2);
        Serial.write((uint8_t *)&corners[i].score, 2);
    }

    Serial.write(FLOW_SYNC, 4);
    uint16_t n_valid = 0;
    for (int i = 0; i < n_flow; i++) {
        if (flow[i].valid) n_valid++;
    }
    Serial.write((uint8_t *)&n_valid, 2);
    for (int i = 0; i < n_flow; i++) {
        if (!flow[i].valid) continue;
        Serial.write((uint8_t *)&flow[i].px, 4);
        Serial.write((uint8_t *)&flow[i].py, 4);
        Serial.write((uint8_t *)&flow[i].dx, 4);
        Serial.write((uint8_t *)&flow[i].dy, 4);
    }
    Serial.flush();
}

// ─── Test capture step ───────────────────────────────────────────────────
static void test_capture_step() {
    camera_fb_t *fb = capture_frame();
    if (!fb) { ESP_LOGE(TAG, "Capture failed!"); return; }

    Corner corners[MAX_CORNERS];
    int n_raw = fast9_detect(fb->buf, fb->width, fb->height, FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);

    ESP_LOGI(TAG, "  FAST: %d corners", n_final);

    // Corner gate
    if (n_final < MIN_CORNERS) {
        ESP_LOGW(TAG, "  LOW CORNERS (%d < %d) — skipped", n_final, MIN_CORNERS);
        send_frame_data(fb, corners, n_final, nullptr, 0, test_frame_num, nullptr);
        memcpy(prev_frame, fb->buf, FRAME_BYTES);
        prev_n_corners = 0;
        has_prev_frame = true;
        esp_camera_fb_return(fb);
        test_frame_num++;
        ESP_LOGI(TAG, "  >>> Improve lighting or point at textured surface");
        return;
    }

    FlowVector flow[MAX_FLOW];
    int n_flow = 0;
    FlowDecomposition decomp = {};
    bool has_flow = false;

    if (has_prev_frame && prev_n_corners > 0) {
        int n_tracked = lk_optical_flow(prev_frame, fb->buf, FRAME_W, FRAME_H,
                                         prev_corners, prev_n_corners, flow);
        n_flow = prev_n_corners;
        flow_accum.add_frame(flow, n_flow, &decomp, n_final);
        has_flow = true;

        int idx = test_frame_num - 1;
        if (idx >= 0 && idx < ACCUM_WINDOW) {
            test_lat_dx[idx] = decomp.lateral_dx;
            test_lat_dy[idx] = decomp.lateral_dy;
            test_radial[idx] = decomp.radial_mean;
            test_vectors[idx] = decomp.n_valid;
            test_stationary[idx] = decomp.is_stationary;
            test_corners[idx] = n_final;
        }

        MotionDirection dir = FlowAccumulator::classify_direction(
            decomp.lateral_dx, decomp.lateral_dy, decomp.radial_mean);

        ESP_LOGI(TAG, "  LK: %d/%d tracked (%d passed, %d rejected)",
                 n_tracked, prev_n_corners, decomp.n_valid, decomp.n_rejected);

        if (decomp.is_stationary) {
            ESP_LOGI(TAG, "  RESULT: STATIONARY");
        } else {
            ESP_LOGI(TAG, "  LATERAL: dx=%+.2f dy=%+.2f px", decomp.lateral_dx, decomp.lateral_dy);
            ESP_LOGI(TAG, "  RADIAL:  %+.2f", decomp.radial_mean);
            ESP_LOGI(TAG, "  DIRECTION: %s", FlowAccumulator::direction_name(dir));
        }

        float tot_dx, tot_dy, tot_rad;
        flow_accum.get_pixel_totals(tot_dx, tot_dy, tot_rad);
        ESP_LOGI(TAG, "  Running total: lat(%+.2f,%+.2f) rad=%+.2f", tot_dx, tot_dy, tot_rad);
    } else {
        ESP_LOGI(TAG, "  Frame 0 = baseline (%d corners)", n_final);
    }

    ESP_LOGI(TAG, "  Sending frame %d...", test_frame_num);
    send_frame_data(fb, corners, n_final, flow, n_flow,
                    test_frame_num, has_flow ? &decomp : nullptr);

    memcpy(prev_frame, fb->buf, FRAME_BYTES);
    memcpy(prev_corners, corners, sizeof(Corner) * n_final);
    prev_n_corners = n_final;
    has_prev_frame = true;
    esp_camera_fb_return(fb);
    test_frame_num++;

    if (flow_accum.is_ready()) {
        CameraMeasurement cm = flow_accum.get_measurement(esp_timer_get_time());

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "======================================================");
        ESP_LOGI(TAG, "   TEST COMPLETE — %d FRAMES (%d active)",
                 ACCUM_WINDOW, cm.n_active_frames);
        ESP_LOGI(TAG, "======================================================");
        for (int i = 0; i < ACCUM_WINDOW - 1; i++) {
            if (test_stationary[i]) {
                ESP_LOGI(TAG, "    F%d: STATIONARY  [%d corners, %d vec]",
                         i + 1, test_corners[i], test_vectors[i]);
            } else {
                MotionDirection fd = FlowAccumulator::classify_direction(
                    test_lat_dx[i], test_lat_dy[i], test_radial[i]);
                ESP_LOGI(TAG, "    F%d: lat(%+6.2f,%+6.2f) rad=%+6.2f  %-11s [%d corners, %d vec]",
                         i + 1, test_lat_dx[i], test_lat_dy[i], test_radial[i],
                         FlowAccumulator::direction_name(fd),
                         test_corners[i], test_vectors[i]);
            }
        }
        ESP_LOGI(TAG, "------------------------------------------------------");
        ESP_LOGI(TAG, "  LATERAL:    dx=%+.2f  dy=%+.2f px", cm.lateral_dx, cm.lateral_dy);
        ESP_LOGI(TAG, "  RADIAL:     %+.2f px", cm.radial_total);
        ESP_LOGI(TAG, "  DIRECTION:  %s", FlowAccumulator::direction_name(cm.direction));
        ESP_LOGI(TAG, "  CONFIDENCE: %.2f", cm.confidence);
        ESP_LOGI(TAG, "======================================================");

        flow_accum.reset();
        has_prev_frame = false;
        test_frame_num = 0;
        ESP_LOGI(TAG, "Press any key for another test, or 't' to exit");
    } else {
        int rem = ACCUM_WINDOW - flow_accum.frame_count();
        ESP_LOGI(TAG, "  >>> Move camera, press any key (%d remaining)", rem);
    }
}

// ─── Public API ──────────────────────────────────────────────────────────

bool cameraInit() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_QVGA;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED");
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_whitebal(s, 1);
    }

    prev_frame = (uint8_t *)heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
    if (!prev_frame) {
        ESP_LOGE(TAG, "Failed to allocate prev_frame!");
        return false;
    }

    flow_accum.reset();
    ESP_LOGI(TAG, "Camera init OK - 320x240 grayscale (stabilized)");
    ESP_LOGI(TAG, "Filters: min_flow=%.1fpx dead_zone=%.1fpx stationary=%.1fpx min_corners=%d",
             MIN_FLOW_MAGNITUDE, FRAME_DEAD_ZONE, STATIONARY_THRESHOLD, MIN_CORNERS);
    ESP_LOGI(TAG, "Directions: FORWARD, BACKWARD, LEFT, RIGHT, STATIONARY");
    ESP_LOGI(TAG, "Commands: 't'=manual test, 'r'=reset");
    return true;
}

// Old interface — backward compatible with main.cpp
// dx = lateral_dx (horizontal pixels, + = right)
// dy = lateral_dy (vertical pixels, + = down)
// confidence = 0.0 - 1.0
// Direction and radial are logged to serial + stored in last_measurement
bool cameraProcessFrame(float &dx, float &dy, float &confidence) {
    if (test_mode) return false;

    camera_fb_t *fb = capture_frame();
    if (!fb) return false;

    Corner corners[MAX_CORNERS];
    int n_raw = fast9_detect(fb->buf, fb->width, fb->height, FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);

    // Corner gate
    if (n_final < MIN_CORNERS) {
        ESP_LOGW(TAG, "Low corners (%d) — skipping", n_final);
        memcpy(prev_frame, fb->buf, FRAME_BYTES);
        prev_n_corners = 0;
        has_prev_frame = true;
        esp_camera_fb_return(fb);
        return false;
    }

    bool measurement_ready = false;

    if (has_prev_frame && prev_n_corners > 0) {
        FlowVector flow[MAX_FLOW];
        lk_optical_flow(prev_frame, fb->buf, FRAME_W, FRAME_H,
                        prev_corners, prev_n_corners, flow);

        FlowDecomposition decomp;
        flow_accum.add_frame(flow, prev_n_corners, &decomp, n_final);

        if (flow_accum.is_ready()) {
            CameraMeasurement cm = flow_accum.get_measurement(esp_timer_get_time());

            if (cm.valid) {
                // FIXED: both outputs are lateral (same coordinate space)
                dx = cm.lateral_dx;         // horizontal pixels
                dy = cm.lateral_dy;         // vertical pixels
                confidence = cm.confidence;
                last_measurement = cm;      // Store full struct for anyone who needs it
                measurement_ready = true;

                ESP_LOGI(TAG, ">>> CAM: %s | lat(%+.1f,%+.1f) rad=%+.1f | conf=%.2f | active=%d/%d",
                         FlowAccumulator::direction_name(cm.direction),
                         cm.lateral_dx, cm.lateral_dy,
                         cm.radial_total, cm.confidence,
                         cm.n_active_frames, cm.n_frames);
            }
            flow_accum.reset();
        }
    }

    memcpy(prev_frame, fb->buf, FRAME_BYTES);
    memcpy(prev_corners, corners, sizeof(Corner) * n_final);
    prev_n_corners = n_final;
    has_prev_frame = true;

    esp_camera_fb_return(fb);
    return measurement_ready;
}

// New interface — returns full CameraMeasurement struct
bool cameraProcessFrame(CameraMeasurement &cam) {
    float dx, dy, conf;
    bool ready = cameraProcessFrame(dx, dy, conf);
    if (ready) {
        cam = last_measurement;
    }
    return ready;
}

// Get the last measurement (useful if EKF needs direction/radial separately)
const CameraMeasurement& cameraGetLastMeasurement() {
    return last_measurement;
}

void cameraDebugCheck() {
    if (!Serial.available()) return;
    char cmd = Serial.read();

    if (test_mode && cmd != 't' && cmd != 'T') {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "-- Frame %d --", test_frame_num);
        test_capture_step();
        return;
    }

    switch (cmd) {
        case 't':
        case 'T':
            if (!test_mode) {
                test_mode = true;
                test_frame_num = 0;
                flow_accum.reset();
                has_prev_frame = false;
                memset(test_lat_dx, 0, sizeof(test_lat_dx));
                memset(test_lat_dy, 0, sizeof(test_lat_dy));
                memset(test_radial, 0, sizeof(test_radial));
                memset(test_vectors, 0, sizeof(test_vectors));
                memset(test_stationary, 0, sizeof(test_stationary));
                memset(test_corners, 0, sizeof(test_corners));

                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "======================================================");
                ESP_LOGI(TAG, "   MANUAL TEST MODE (stabilized)");
                ESP_LOGI(TAG, "======================================================");
                ESP_LOGI(TAG, "  Directions: FORWARD, BACKWARD, LEFT, RIGHT");
                ESP_LOGI(TAG, "  Noise rejection: ON");
                ESP_LOGI(TAG, "  Corner gate: min %d corners", MIN_CORNERS);
                ESP_LOGI(TAG, "  Press any key for each frame. 't' to exit.");
                ESP_LOGI(TAG, "  Use test_viewer.py to save frame images.");
                ESP_LOGI(TAG, "======================================================");
                ESP_LOGI(TAG, ">>> Press any key for baseline...");
            } else {
                test_mode = false;
                flow_accum.reset();
                has_prev_frame = false;
                ESP_LOGI(TAG, "Test mode OFF. Pipeline resumed.");
            }
            break;

        case 'r':
        case 'R':
            flow_accum.reset();
            has_prev_frame = false;
            test_frame_num = 0;
            ESP_LOGI(TAG, "RESET.");
            break;

        default:
            break;
    }
}

} // namespace drift