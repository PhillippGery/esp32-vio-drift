//panchitos space

#include "vio_camera.h"
#include <Arduino.h>
#include <cstring>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
// Assume these headers exist in your project:
#include "camera_pins.h"
#include "fast_corner.h"
#include "optical_flow.h"
#include "flow_accumulator.h"

namespace drift {

static const char *TAG = "CAM_VIO";

// ─── Constants & State Variables ─────────────────────────────────────────
static constexpr int FAST_THRESHOLD = 20;
static constexpr int NMS_RADIUS     = 8;
static constexpr int FRAME_W = 320;
static constexpr int FRAME_H = 240;
static constexpr int FRAME_BYTES = FRAME_W * FRAME_H;

static constexpr float FOCAL_LENGTH_PX  = 240.0f;  // TODO: update after calibration
static constexpr float BASELINE_DEPTH_M = 0.5f;

static uint8_t *prev_frame = nullptr;
static Corner   prev_corners[MAX_CORNERS];
static int      prev_n_corners = 0;
static bool     has_prev_frame = false;

static FlowAccumulator flow_accum;

// Sync markers — must match Python scripts (save_frame, view_corners, view_flow, live_view)
// to call live_view
// python .\60_scripts\visualization\live_view.py --port COM5
static const uint8_t FRAME_SYNC[]  = {0xFF, 0xD8, 0xBE, 0xEF};
static const uint8_t CORNER_SYNC[] = {0xC0, 0x52, 0x4E, 0x52};
static const uint8_t FLOW_SYNC[]   = {0xF1, 0x0E, 0xDA, 0x7A};

// ─── Internal Helper Functions ───────────────────────────────────────────
static camera_fb_t* capture_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) ESP_LOGE(TAG, "Capture failed");
    return fb;
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
        ESP_LOGE(TAG, "Failed to allocate prev_frame in PSRAM!");
        return false;
    }

    flow_accum.reset();
    ESP_LOGI(TAG, "Camera init OK - 320x240 grayscale, PSRAM double-buffered");
    return true;
}

bool cameraProcessFrame(float &dx, float &dy, float &confidence) {
    int64_t t0 = esp_timer_get_time();
    camera_fb_t *fb = capture_frame();
    if (!fb) return false;
    float t_cap = (esp_timer_get_time() - t0) / 1000.0f;

    // STAGE 1: FAST corners
    Corner corners[MAX_CORNERS];
    int64_t t1 = esp_timer_get_time();
    int n_raw   = fast9_detect(fb->buf, fb->width, fb->height, FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);
    float t_fast = (esp_timer_get_time() - t1) / 1000.0f;

    bool measurement_ready = false;
    float t_lk = 0;
    int n_valid_flow = 0;

    // STAGE 2: Lucas-Kanade optical flow
    if (has_prev_frame && prev_n_corners > 0) {
        FlowVector flow[MAX_FLOW];
        int64_t t2 = esp_timer_get_time();
        n_valid_flow = lk_optical_flow(prev_frame, fb->buf, FRAME_W, FRAME_H,
                                        prev_corners, prev_n_corners, flow);
        t_lk = (esp_timer_get_time() - t2) / 1000.0f;

        // STAGE 3: Accumulate flow
        flow_accum.add_frame(flow, prev_n_corners);

        // STAGE 4: Produce VIO measurement if window is full
        if (flow_accum.is_ready()) {
            VioMeasurement vio = flow_accum.to_vio_measurement(
                FOCAL_LENGTH_PX, BASELINE_DEPTH_M, esp_timer_get_time());

            if (vio.valid) {
                dx = vio.delta_x_m;
                dy = vio.delta_y_m;
                confidence = vio.confidence;
                measurement_ready = true;
            }
            flow_accum.reset();
        }
    }

    // Log pipeline timing
    ESP_LOGI(TAG, "cap:%.1f FAST:%.1f LK:%.1f ms | corners:%d flow:%d accum:%d/%d",
             t_cap, t_fast, t_lk, n_final, n_valid_flow,
             flow_accum.frame_count(), ACCUM_WINDOW);

    // Store current as previous
    memcpy(prev_frame, fb->buf, FRAME_BYTES);
    memcpy(prev_corners, corners, sizeof(Corner) * n_final);
    prev_n_corners = n_final;
    has_prev_frame = true;

    esp_camera_fb_return(fb);
    return measurement_ready;
}

void cameraDebugCheck() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    if (cmd != 'c' && cmd != 'C') return;

    ESP_LOGI(TAG, "Debug capture triggered");

    camera_fb_t *fb = capture_frame();
    if (!fb) return;

    // FAST corners
    Corner corners[MAX_CORNERS];
    int n_raw = fast9_detect(fb->buf, fb->width, fb->height, FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);

    // LK flow
    FlowVector flow[MAX_FLOW];
    int n_flow_total = 0;
    if (has_prev_frame && prev_n_corners > 0) {
        lk_optical_flow(prev_frame, fb->buf, FRAME_W, FRAME_H,
                        prev_corners, prev_n_corners, flow);
        n_flow_total = prev_n_corners;
    }

    // Send frame + corners + flow over serial for Python tools
    vTaskDelay(pdMS_TO_TICKS(50));

    // 1. Frame
    Serial.write(FRAME_SYNC, 4);
    Serial.write(fb->buf, fb->len);

    // 2. Corners
    Serial.write(CORNER_SYNC, 4);
    uint16_t cc = (uint16_t)n_final;
    Serial.write((uint8_t *)&cc, 2);
    for (int i = 0; i < n_final; i++) {
        Serial.write((uint8_t *)&corners[i].x, 2);
        Serial.write((uint8_t *)&corners[i].y, 2);
        Serial.write((uint8_t *)&corners[i].score, 2);
    }

    // 3. Flow vectors (only valid)
    Serial.write(FLOW_SYNC, 4);
    uint16_t n_valid = 0;
    for (int i = 0; i < n_flow_total; i++) {
        if (flow[i].valid) n_valid++;
    }
    Serial.write((uint8_t *)&n_valid, 2);
    for (int i = 0; i < n_flow_total; i++) {
        if (!flow[i].valid) continue;
        Serial.write((uint8_t *)&flow[i].px, 4);
        Serial.write((uint8_t *)&flow[i].py, 4);
        Serial.write((uint8_t *)&flow[i].dx, 4);
        Serial.write((uint8_t *)&flow[i].dy, 4);
    }
    Serial.flush();

    // Update previous frame
    memcpy(prev_frame, fb->buf, FRAME_BYTES);
    memcpy(prev_corners, corners, sizeof(Corner) * n_final);
    prev_n_corners = n_final;
    has_prev_frame = true;

    esp_camera_fb_return(fb);
    ESP_LOGI(TAG, "Debug frame sent: %d corners, %d flow", n_final, n_valid);
}

} // namespace drift