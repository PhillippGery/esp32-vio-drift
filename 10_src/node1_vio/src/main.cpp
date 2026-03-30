// ============================================================
// PROJECT DRIFT — Node 1: Camera Capture & FAST Pipeline
// Board: Seeed XIAO ESP32S3 Sense (OV3660)
// Framework: Arduino (ESP-IDF APIs used throughout)
//
// NOTE FOR TEAM: The Arduino-ESP32 framework IS ESP-IDF under
// the hood. This code uses ESP-IDF APIs exclusively:
//   - ESP_LOGI / ESP_LOGE / ESP_LOGW  (logging)
//   - esp_timer_get_time()            (microsecond timer)
//   - vTaskDelay / pdMS_TO_TICKS      (FreeRTOS scheduling)
//   - heap_caps_get_free_size()       (memory introspection)
//   - xTaskCreatePinnedToCore()       (dual-core task pinning)
// The only Arduino remnant is the setup()/loop() entry point.
// ============================================================

#include <Arduino.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_pins.h"
#include "fast_corner.h"

static const char *TAG = "DRIFT";

// FAST detection tuning
static constexpr int FAST_THRESHOLD = 40; //edit this value depending on contrast
static constexpr int NMS_RADIUS     = 8;

// Sync markers — must match Python scripts
static const uint8_t FRAME_SYNC[]  = {0xFF, 0xD8, 0xBE, 0xEF};
static const uint8_t CORNER_SYNC[] = {0xC0, 0x52, 0x4E, 0x52};

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
    config.frame_size   = FRAMESIZE_QVGA;       // 320x240
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: 0x%x", err);
        return false;
    }

    // Tune sensor for indoor VIO
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_whitebal(s, 1);
    }

    ESP_LOGI(TAG, "Camera init OK - 320x240 grayscale, double-buffered in PSRAM");
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
// Send frame + corner data over serial (for view_corners.py)
// ------------------------------------------------------------
static void send_frame_and_corners(camera_fb_t *fb, Corner *corners,
                                   int n_corners, float t_cap_ms,
                                   float t_fast_ms) {
    ESP_LOGI(TAG, "Sending frame + %d corners...", n_corners);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 1. Frame data
    Serial.write(FRAME_SYNC, 4);
    Serial.write(fb->buf, fb->len);

    // 2. Corner data: SYNC(4) + count(2) + [x(2) + y(2) + score(2)] * count
    Serial.write(CORNER_SYNC, 4);
    uint16_t count = (uint16_t)n_corners;
    Serial.write((uint8_t *)&count, 2);
    for (int i = 0; i < n_corners; i++) {
        Serial.write((uint8_t *)&corners[i].x, 2);
        Serial.write((uint8_t *)&corners[i].y, 2);
        Serial.write((uint8_t *)&corners[i].score, 2);
    }
    Serial.flush();

    ESP_LOGI(TAG, "Done. cap=%.1fms FAST=%.1fms corners=%d",
             t_cap_ms, t_fast_ms, n_corners);
}

// ------------------------------------------------------------
// setup() — runs once on Core 1
// ------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PROJECT DRIFT - Node 1 Camera Init");
    ESP_LOGI(TAG, "========================================");

    // Check PSRAM
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_total > 0) {
        ESP_LOGI(TAG, "PSRAM: %u / %u bytes free", psram_free, psram_total);
    } else {
        ESP_LOGW(TAG, "No PSRAM detected - frame buffers will use internal RAM!");
    }

    // Init camera
    if (!init_camera()) {
        ESP_LOGE(TAG, "Camera init failed - halting.");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    ESP_LOGI(TAG, "Ready. Starting capture loop...");
    ESP_LOGI(TAG, "Send 'c' over serial to capture frame + corners");
}

// ------------------------------------------------------------
// loop() — runs continuously on Core 1
// ------------------------------------------------------------
void loop() {
    // Check for serial command
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            int64_t t0 = esp_timer_get_time();
            camera_fb_t *fb = capture_frame();
            if (fb) {
                float t_cap = (esp_timer_get_time() - t0) / 1000.0f;

                Corner corners[MAX_CORNERS];
                int64_t t1 = esp_timer_get_time();
                int n_raw   = fast9_detect(fb->buf, fb->width, fb->height,
                                           FAST_THRESHOLD, corners);
                int n_final = fast_nms(corners, n_raw, NMS_RADIUS);
                float t_fast = (esp_timer_get_time() - t1) / 1000.0f;

                send_frame_and_corners(fb, corners, n_final, t_cap, t_fast);
                esp_camera_fb_return(fb);
            }
            return;
        }
    }

    // Normal loop: capture + FAST + log timing
    int64_t t0 = esp_timer_get_time();
    camera_fb_t *fb = capture_frame();
    if (!fb) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    float t_cap = (esp_timer_get_time() - t0) / 1000.0f;

    Corner corners[MAX_CORNERS];
    int64_t t1 = esp_timer_get_time();
    int n_raw   = fast9_detect(fb->buf, fb->width, fb->height,
                               FAST_THRESHOLD, corners);
    int n_final = fast_nms(corners, n_raw, NMS_RADIUS);
    float t_fast = (esp_timer_get_time() - t1) / 1000.0f;

    ESP_LOGI(TAG, "cap: %.1f ms | FAST: %.1f ms | corners: %d raw -> %d nms",
             t_cap, t_fast, n_raw, n_final);

    // Log first 5 corners
    int n_print = (n_final < 5) ? n_final : 5;
    for (int i = 0; i < n_print; i++) {
        ESP_LOGI(TAG, "  [%d] (%d, %d) score=%d",
                 i, corners[i].x, corners[i].y, corners[i].score);
    }

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(50));
}
