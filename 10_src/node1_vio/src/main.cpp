// ============================================================
// PROJECT DRIFT — Node 1: Full VIO Pipeline
// Board: Seeed XIAO ESP32S3 Sense (OV3660)
// Framework: Arduino (ESP-IDF APIs used throughout)
//
// Pipeline: CAPTURE -> FAST -> LK Flow -> Accumulate -> EKF
//
// The flow accumulator averages optical flow over N frames
// to produce stable displacement estimates for the EKF.
//
// EKF State Vector (12-DOF, from Phillipp):
//   [px, py, pz, vx, vy, vz, roll, pitch, yaw, bx, by, bz]
//   Camera updates: px, py (via accumulated pixel displacement)
//   IMU updates:    all 12 states
// ============================================================

#include <Arduino.h>
#include <cstring>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_pins.h"
#include "fast_corner.h"
#include "optical_flow.h"
#include "flow_accumulator.h"

static const char *TAG = "DRIFT";

// FAST detection tuning
static constexpr int FAST_THRESHOLD = 20;
static constexpr int NMS_RADIUS     = 8;

// Frame dimensions
static constexpr int FRAME_W = 320;
static constexpr int FRAME_H = 240;
static constexpr int FRAME_BYTES = FRAME_W * FRAME_H;

// Camera intrinsics (approximate for OV3660 at 320x240)
// TODO: Calibrate these with a checkerboard pattern
static constexpr float FOCAL_LENGTH_PX  = 240.0f;  // Estimated focal length in pixels
static constexpr float BASELINE_DEPTH_M = 0.5f;    // Assumed scene depth in meters

// Previous frame buffer (allocated in PSRAM) and corner storage
static uint8_t *prev_frame = nullptr;
static Corner   prev_corners[MAX_CORNERS];
static int      prev_n_corners = 0;
static bool     has_prev_frame = false;

// Flow accumulator instance
static FlowAccumulator flow_accum;

// Sync markers — must match Python scripts
static const uint8_t FRAME_SYNC[]  = {0xFF, 0xD8, 0xBE, 0xEF};
static const uint8_t CORNER_SYNC[] = {0xC0, 0x52, 0x4E, 0x52};
static const uint8_t FLOW_SYNC[]   = {0xF1, 0x0E, 0xDA, 0x7A};
static const uint8_t VIO_SYNC[]    = {0xA1, 0xB2, 0xC3, 0xD4};

// ------------------------------------------------------------
// Camera Configuration
// ------------------------------------------------------------
static bool init_camera(void) {
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

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: 0x%x", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_whitebal(s, 1);
    }

    ESP_LOGI(TAG, "Camera init OK - 320x240 grayscale, double-buffered");
    return true;
}

// ------------------------------------------------------------
// Frame capture
// ------------------------------------------------------------
static camera_fb_t* capture_frame(void) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Capture failed");
    }
    return fb;
}

// ------------------------------------------------------------
// Send frame + corners + flow + VIO measurement over serial
// ------------------------------------------------------------
static void send_pipeline_data(camera_fb_t *fb,
                               Corner *corners, int n_corners,
                               FlowVector *flow, int n_flow_total,
                               const VioMeasurement *vio,
                               float t_cap, float t_fast, float t_lk) {
    ESP_LOGI(TAG, "Sending frame + %d corners + flow...", n_corners);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 1. Frame
    Serial.write(FRAME_SYNC, 4);
    Serial.write(fb->buf, fb->len);

    // 2. Corners
    Serial.write(CORNER_SYNC, 4);
    uint16_t cc = (uint16_t)n_corners;
    Serial.write((uint8_t *)&cc, 2);
    for (int i = 0; i < n_corners; i++) {
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

    // 4. VIO measurement (accumulated)
    Serial.write(VIO_SYNC, 4);
    uint8_t vio_valid = vio->valid ? 1 : 0;
    Serial.write(&vio_valid, 1);
    Serial.write((uint8_t *)&vio->delta_x_m, 4);
    Serial.write((uint8_t *)&vio->delta_y_m, 4);
    Serial.write((uint8_t *)&vio->confidence, 4);

    Serial.flush();
}

// ------------------------------------------------------------
// setup()
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PROJECT DRIFT - Node 1 VIO Pipeline");
    ESP_LOGI(TAG, "========================================");

    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_total > 0) {
        ESP_LOGI(TAG, "PSRAM: %u / %u bytes free", psram_free, psram_total);
    } else {
        ESP_LOGW(TAG, "No PSRAM detected!");
    }

    if (!init_camera()) {
        ESP_LOGE(TAG, "Camera init failed - halting.");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    prev_frame = (uint8_t *)heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
    if (!prev_frame) {
        ESP_LOGE(TAG, "Failed to allocate prev_frame in PSRAM!");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    flow_accum.reset();

    ESP_LOGI(TAG, "Pipeline: CAPTURE -> FAST -> LK -> ACCUM(%d) -> EKF",
             ACCUM_WINDOW);
    ESP_LOGI(TAG, "Camera: f=%.0fpx depth=%.1fm", FOCAL_LENGTH_PX, BASELINE_DEPTH_M);
    ESP_LOGI(TAG, "Send 'c' for visualization capture");
}

// ------------------------------------------------------------
// loop() — full VIO pipeline with flow accumulation
// ------------------------------------------------------------
void loop() {
    // ---- Serial command: visualization capture ----
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            camera_fb_t *fb = capture_frame();
            if (!fb) return;

            Corner corners[MAX_CORNERS];
            int64_t t1 = esp_timer_get_time();
            int n_raw = fast9_detect(fb->buf, fb->width, fb->height,
                                     FAST_THRESHOLD, corners);
            int n_final = fast_nms(corners, n_raw, NMS_RADIUS);
            float t_fast = (esp_timer_get_time() - t1) / 1000.0f;

            FlowVector flow[MAX_FLOW];
            int n_flow_total = 0;
            float t_lk = 0;
            if (has_prev_frame && prev_n_corners > 0) {
                int64_t t2 = esp_timer_get_time();
                lk_optical_flow(prev_frame, fb->buf,
                                FRAME_W, FRAME_H,
                                prev_corners, prev_n_corners, flow);
                t_lk = (esp_timer_get_time() - t2) / 1000.0f;
                n_flow_total = prev_n_corners;
            }

            // Get current VIO measurement
            VioMeasurement vio = flow_accum.to_vio_measurement(
                FOCAL_LENGTH_PX, BASELINE_DEPTH_M, esp_timer_get_time());

            send_pipeline_data(fb, corners, n_final,
                              flow, n_flow_total, &vio,
                              0, t_fast, t_lk);

            memcpy(prev_frame, fb->buf, FRAME_BYTES);
            memcpy(prev_corners, corners, sizeof(Corner) * n_final);
            prev_n_corners = n_final;
            has_prev_frame = true;

            esp_camera_fb_return(fb);
            return;
        }
    }

    // ---- Normal pipeline loop ----
    int64_t t0 = esp_timer_get_time();
    camera_fb_t *fb = capture_frame();
    if (!fb) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    float t_cap = (esp_timer_get_time() - t0) / 1000.0f;

    // STAGE 1: FAST corners
    Corner corners[MAX_CORNERS];
    int64_t t1 = esp_timer_get_time();
    int n_raw   = fast9_detect(fb->buf, fb->width, fb->height,
                               FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);
    float t_fast = (esp_timer_get_time() - t1) / 1000.0f;

    // STAGE 2: Lucas-Kanade optical flow
    float t_lk = 0;
    int n_valid_flow = 0;
    if (has_prev_frame && prev_n_corners > 0) {
        FlowVector flow[MAX_FLOW];
        int64_t t2 = esp_timer_get_time();
        n_valid_flow = lk_optical_flow(prev_frame, fb->buf,
                                       FRAME_W, FRAME_H,
                                       prev_corners, prev_n_corners,
                                       flow);
        t_lk = (esp_timer_get_time() - t2) / 1000.0f;

        // STAGE 3: Accumulate flow
        flow_accum.add_frame(flow, prev_n_corners);

        // STAGE 4: When accumulator is ready, produce VIO measurement
        if (flow_accum.is_ready()) {
            VioMeasurement vio = flow_accum.to_vio_measurement(
                FOCAL_LENGTH_PX, BASELINE_DEPTH_M, esp_timer_get_time());

            if (vio.valid) {
                ESP_LOGI(TAG, "VIO: dx=%.4fm dy=%.4fm conf=%.2f (%d frames)",
                         vio.delta_x_m, vio.delta_y_m,
                         vio.confidence, flow_accum.frame_count());

                // -------------------------------------------------------
                // >>> EKF UPDATE POINT <<<
                // Pass vio.delta_x_m, vio.delta_y_m to Phillipp's EKF
                // as the visual measurement updating [px, py] in state:
                //   [px, py, pz, vx, vy, vz, roll, pitch, yaw, bx, by, bz]
                //
                // vio.confidence can scale the measurement noise R matrix:
                //   R = base_R / vio.confidence
                //   (higher confidence = trust camera more)
                // -------------------------------------------------------
            }

            // Reset accumulator for next window
            flow_accum.reset();
        }
    }

    ESP_LOGI(TAG, "cap:%.1f FAST:%.1f LK:%.1f ms | corners:%d flow:%d accum:%d/%d",
             t_cap, t_fast, t_lk, n_final, n_valid_flow,
             flow_accum.frame_count(), ACCUM_WINDOW);

    // Store current as previous
    memcpy(prev_frame, fb->buf, FRAME_BYTES);
    memcpy(prev_corners, corners, sizeof(Corner) * n_final);
    prev_n_corners = n_final;
    has_prev_frame = true;

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(50));
}