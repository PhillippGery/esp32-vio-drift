/**
 * @file    main.cpp
 * @brief   PROJECT DRIFT — Node 1 VIO Firmware
 *          ESP32 Feather V2 + MPU-6050 + ArduCAM Mini SPI
 *
 * Pipeline overview:
 *  1. Read IMU at 200 Hz → pre-filter (complementary / Madgwick)
 *  2. Capture ArduCAM frame on trigger (feature extraction later)
 *  3. Run Extended Kalman Filter: predict (IMU) + update (camera features)
 *  4. Transmit JSON odometry packet over WiFi UDP at 50 Hz
 *
 * @author  Phillipp Gery (Kalman), Panchtio (Camera), Sam (Firmware)
 */


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "ekf.h"
#include "status_led.h"
#include "vio_camera.h"
#include "mpu6050.h"

// TODO (Panchtio): include ArduCAM headers
// TODO (Phillipp): include EKF header from include/ekf.h


// ── I2C Pins for XIAO ESP32S3 ─────────────────────────────────────────────
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 5  
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 6
#endif


// ── Task periods (ms) ─────────────────────────────────────────────────────
constexpr uint32_t IMU_PERIOD_MS   = 5;   // 200 Hz
constexpr uint32_t TX_PERIOD_MS    = 20;  // 50 Hz
constexpr uint32_t CAM_PERIOD_MS   = 200; // 5 Hz (placeholder)

// ── Timing trackers ───────────────────────────────────────────────────────
static uint32_t lastImuMs  = 0;
static uint32_t lastTxMs   = 0;
static uint32_t lastCamMs  = 0;

MPU6050 imu;

static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
static const char* UDP_TARGET_IP = "255.255.255.255";
constexpr uint16_t UDP_TARGET_PORT = 4210;
constexpr uint16_t UDP_LOCAL_PORT = 4211;

static WiFiUDP udp;

static bool startWiFi() {
    Serial.printf("[NODE %d] Connecting to WiFi SSID='%s'...\n", NODE_ID, WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t timeout_ms = 15000;
    uint32_t start_ms = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_ms < timeout_ms) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[NODE 1] WiFi connect failed.");
        return false;
    }

    Serial.printf("[NODE %d] WiFi connected, IP=%s\n", NODE_ID, WiFi.localIP().toString().c_str());
    return true;
}

static void sendOdometryPacket(uint32_t timestamp_ms) {
    float px, py, pz, roll, pitch, yaw;
    drift::ekfGetPosition(px, py, pz);
    drift::ekfGetOrientation(roll, pitch, yaw);

    StaticJsonDocument<256> packet;
    packet["node"] = NODE_ID;
    packet["ts"] = timestamp_ms;
    packet["px"] = px;
    packet["py"] = py;
    packet["pz"] = pz;
    packet["roll"] = roll;
    packet["pitch"] = pitch;
    packet["yaw"] = yaw;

    char buffer[256];
    size_t len = serializeJson(packet, buffer, sizeof(buffer));
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(buffer), len);
    udp.endPacket();
}

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.printf("[NODE %d] PROJECT DRIFT — VIO Node booting...\n", NODE_ID);

    // Initialize I2C for the MPU-6050
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    imu.begin(); // Default I2C address 0x68, can be changed if needed
    // Calibrate IMU (stationary, flat surface)
    // Init imu
    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n",imu.offsetAx, imu.offsetAy, imu.offsetAz);
    
    
    // TODO (Phillipp): EKF init
    // Initialize Kalman Filter Matrices
    drift::ekfInit();



    // TODO (Panchtio): ArduCAM init + test JPEG capture
    // Initialize XIAO OV3660 Camera Pipeline & PSRAM
    if (drift::cameraInit()) {
        Serial.println("[NODE 1] Camera initialized successfully.");
    } else {
        Serial.println("[NODE 1] CRITICAL: Camera init failed! Halting.");
        while (true) { delay(1000); }
    }

    if (!startWiFi()) {
        Serial.println("[NODE 1] CRITICAL: WiFi init failed! Halting.");
        while (true) { delay(1000); }
    }
    udp.begin(UDP_LOCAL_PORT);
    Serial.printf("[NODE %d] UDP transport ready (remote %s:%u)\n", NODE_ID, UDP_TARGET_IP, UDP_TARGET_PORT);


    // drift::ekfInit();
    // drift::ledInit();
    // drift::ledSet(drift::StatusColor::RED);

    
    Serial.println("[NODE 1] Setup complete.");
}

// ------------------------------------------------------------
// loop() — full VIO pipeline with flow accumulation
// ------------------------------------------------------------
void loop() {
    uint32_t now = millis();


    

    // ── IMU read ─────────────────────────────────────────────────────────
    if (now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs = now;
        // TODO (Phillipp): readIMU() → EKF predict step
        float ax, ay, az, gx, gy, gz;
        imu.read(ax, ay, az, gx, gy, gz);
        //Serial.printf("A: %.3f  %.3f  %.3f  |  G: %.3f  %.3f  %.3f\n", x, ay, az, gx, gy, gz);
        delay(20);
        
    }

    // ── Camera capture ───────────────────────────────────────────────────
    if (now - lastCamMs >= CAM_PERIOD_MS) {
        lastCamMs = now;
        // TODO (Panchtio): captureFrame() → feature extraction → EKF update

        float dx, dy, confidence;
        // If the optical flow accumulator finishes a window:
        if (drift::cameraProcessFrame(dx, dy, confidence)) {
            // EKF UPDATE: Feed the visual displacement to the math engine!
            drift::ekfUpdateCamera(dx, dy); 
            printf("[NODE 1] EKF updated with camera measurement: dx=%.3f m, dy=%.3f m, confidence=%.2f\n", dx, dy, confidence);
        }
    }

    // ── Debug triggers (Catch 'c' commands from Python) ──────────────────
    drift::cameraDebugCheck();

    // ── Telemetry transmit ───────────────────────────────────────────────
    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        sendOdometryPacket(now);
        Serial.printf("[NODE %d] Sent odometry packet at %u ms\n", NODE_ID, now);
    }
}

