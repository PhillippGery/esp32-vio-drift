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
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include "ekf.h"
#include "status_led.h"
#include "vio_camera.h"

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

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.printf("[NODE %d] PROJECT DRIFT — VIO Node booting...\n", NODE_ID);

    // Initialize I2C for the MPU-6050
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    
    // Initialize Kalman Filter Matrices
    drift::ekfInit();
    
    // Initialize XIAO OV3660 Camera Pipeline & PSRAM
    if (drift::cameraInit()) {
        Serial.println("[NODE 1] Camera initialized successfully.");
    } else {
        Serial.println("[NODE 1] CRITICAL: Camera init failed! Halting.");
        while (true) { delay(1000); }
    }

    // TODO (Sam):    WiFi + UDP socket init
    // TODO (Phillipp): EKF init

    // drift::ekfInit();
    // drift::ledInit();
    // drift::ledSet(drift::StatusColor::RED);

    // TODO (Panchtio): ArduCAM init + test JPEG capture




    Serial.println("[NODE 1] Setup complete.");
}

// ------------------------------------------------------------
// loop() — full VIO pipeline with flow accumulation
// ------------------------------------------------------------
void loop() {
    uint32_t now = millis();


    


    //Serial.println("[NODE 1] Test print in Loop.");

    // ── IMU read ─────────────────────────────────────────────────────────
    if (now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs = now;
        // TODO (Phillipp): readIMU() → EKF predict step
        
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
        // TODO (Sam): serialize EKF state to JSON → UDP send
    }
}
