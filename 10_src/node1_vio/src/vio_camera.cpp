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

static constexpr float FOCAL_LENGTH_PX  = 240.0f; 
static constexpr float BASELINE_DEPTH_M = 0.5f;   

static uint8_t *prev_frame = nullptr;
static Corner   prev_corners[MAX_CORNERS];
static int      prev_n_corners = 0;
static bool     has_prev_frame = false;

static FlowAccumulator flow_accum;

// ─── Internal Helper Functions ───────────────────────────────────────────
static camera_fb_t* capture_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) ESP_LOGE(TAG, "Capture failed");
    return fb;
}

// ─── Public API ──────────────────────────────────────────────────────────

bool cameraInit() {
    // Note: This config assumes the XIAO ESP32S3 pinout from camera_pins.h
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

    prev_frame = (uint8_t *)heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
    if (!prev_frame) {
        ESP_LOGE(TAG, "Failed to allocate prev_frame in PSRAM!");
        return false;
    }

    flow_accum.reset();
    return true;
}

bool cameraProcessFrame(float &dx, float &dy, float &confidence) {
    camera_fb_t *fb = capture_frame();
    if (!fb) return false;

    // STAGE 1: FAST corners
    Corner corners[MAX_CORNERS];
    int n_raw   = fast9_detect(fb->buf, fb->width, fb->height, FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);

    bool measurement_ready = false;

    // STAGE 2: Lucas-Kanade optical flow
    if (has_prev_frame && prev_n_corners > 0) {
        FlowVector flow[MAX_FLOW];
        lk_optical_flow(prev_frame, fb->buf, FRAME_W, FRAME_H, prev_corners, prev_n_corners, flow);

        // STAGE 3: Accumulate flow
        flow_accum.add_frame(flow, prev_n_corners);

        // STAGE 4: Produce VIO measurement if window is full
        if (flow_accum.is_ready()) {
            VioMeasurement vio = flow_accum.to_vio_measurement(FOCAL_LENGTH_PX, BASELINE_DEPTH_M, esp_timer_get_time());
            
            if (vio.valid) {
                dx = vio.delta_x_m;
                dy = vio.delta_y_m;
                confidence = vio.confidence;
                measurement_ready = true;
            }
            flow_accum.reset();
        }
    }

    // Store current as previous
    memcpy(prev_frame, fb->buf, FRAME_BYTES);
    memcpy(prev_corners, corners, sizeof(Corner) * n_final);
    prev_n_corners = n_final;
    has_prev_frame = true;

    esp_camera_fb_return(fb);
    return measurement_ready;
}

void cameraDebugCheck() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            ESP_LOGI(TAG, "Debug visualization requested. (Logic moved/simplified for cleanliness)");
            // If you want to keep the massive Serial.write block for debugging, 
            // put your colleague's visualization logic right here.
        }
    }
}

} // namespace drift