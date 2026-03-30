// ============================================================
// PROJECT DRIFT — Live Camera Stream (Debug Tool)
// Board: Seeed XIAO ESP32S3 Sense (OV3660)
//
// This is a SEPARATE debug firmware for viewing the camera
// feed in your browser. It does NOT run the VIO pipeline.
// Flash main.cpp when you're done previewing.
//
// Usage:
//   1. Flash this file as your main
//   2. Open Serial Monitor — note the IP address
//   3. Open http://<IP_ADDRESS> in your browser
// ============================================================

#include <Arduino.h>
#include "esp_camera.h"
#include "WiFi.h"
#include "esp_http_server.h"
#include "camera_pins.h"

// ---- CHANGE THESE TO YOUR WIFI ----
const char *WIFI_SSID = "Benitezfu";
const char *WIFI_PASS = "Panchito2708";
// ------------------------------------

#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=frame"
#define FRAME_BOUNDARY      "\r\n--frame\r\n"
#define FRAME_HEADER        "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

// Camera init — same pins, but JPEG format for streaming
bool initCamera() {
    camera_config_t config;
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
    config.pixel_format = PIXFORMAT_JPEG;       // JPEG for streaming
    config.frame_size   = FRAMESIZE_QVGA;       // 320x240
    config.jpeg_quality = 12;                    // 0-63, lower = better quality
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init FAILED: 0x%x\n", err);
        return false;
    }
    Serial.println("[CAM] Init OK — JPEG 320x240");
    return true;
}

// MJPEG stream handler — pushes frames continuously
static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    char header_buf[64];

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Capture failed");
            continue;
        }

        // Send boundary
        res = httpd_resp_send_chunk(req, FRAME_BOUNDARY, strlen(FRAME_BOUNDARY));
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Send content-type + length header
        int hlen = snprintf(header_buf, sizeof(header_buf), FRAME_HEADER, fb->len);
        res = httpd_resp_send_chunk(req, header_buf, hlen);
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Send JPEG frame data
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);

        if (res != ESP_OK) break;
    }
    return res;
}

// Simple HTML page with the stream embedded
static esp_err_t index_handler(httpd_req_t *req) {
    const char html[] =
        "<!DOCTYPE html><html><head>"
        "<title>DRIFT Camera</title>"
        "<style>"
        "body{background:#111;color:#fff;font-family:monospace;"
        "display:flex;flex-direction:column;align-items:center;margin-top:30px}"
        "h1{color:#0f0}img{border:2px solid #0f0;margin-top:20px}"
        "</style></head><body>"
        "<h1>PROJECT DRIFT — Live Camera</h1>"
        "<p>XIAO ESP32S3 Sense | OV3660 | 320x240</p>"
        "<img src=\"/stream\" width=\"640\" height=\"480\">"
        "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// Start HTTP server with two endpoints: / and /stream
void startServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET,
                                  .handler = index_handler, .user_ctx = NULL };
        httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET,
                                   .handler = stream_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);
        Serial.println("[HTTP] Server started on port 80");
    } else {
        Serial.println("[HTTP] Server start FAILED");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("  DRIFT — Live Camera Stream (Debug)");
    Serial.println("========================================");

    if (psramFound()) {
        Serial.printf("[SYS] PSRAM: %d bytes free\n", ESP.getFreePsram());
    }

    if (!initCamera()) {
        Serial.println("[SYS] Camera failed — halting.");
        while (true) delay(1000);
    }

    // Connect to WiFi
    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 30) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK!");
        Serial.printf("[WIFI] IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.println("========================================");
        Serial.printf("  Open http://%s in your browser\n", WiFi.localIP().toString().c_str());
        Serial.println("========================================\n");
        startServer();
    } else {
        Serial.println(" FAILED!");
        Serial.println("[WIFI] Check SSID and password.");
    }
}

void loop() {
    delay(10000);  // Nothing to do — stream runs in HTTP server task
}
