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
static uint32_t lastImuMs   = 0;
static uint32_t lastCamMs   = 0;
static uint32_t lastPrintMs = 0;
static float last_cam_yaw   = 0.0f;

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

    drift::ekfGetOrientation(last_cam_yaw);




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
    if (VISION_ENABLED && (now - lastCamMs >= CAM_PERIOD_MS)) {
        float cam_dt = (now - lastCamMs) / 1000.0f; 
        lastCamMs = now;
        
        // 1. Calculate how much the IMU rotated since the last camera frame
        float current_yaw;
        drift::ekfGetOrientation(current_yaw);
        
        // Calculate shortest path angular difference to prevent 360-degree wrap-around bugs
        float delta_yaw_imu = current_yaw - last_cam_yaw;
        while (delta_yaw_imu > M_PI)  delta_yaw_imu -= 2.0f * M_PI;
        while (delta_yaw_imu < -M_PI) delta_yaw_imu += 2.0f * M_PI;
        
        last_cam_yaw = current_yaw; // Save for next frame
        
        float meas_vx, meas_vy, confidence;
        
        // 2. Pass the dt AND the rotation fix into the camera processor
        if (drift::cameraProcessFrame(meas_vx, meas_vy, confidence, cam_dt, delta_yaw_imu)) {
            
            // 3. Fuse the metric, derotated anchor into the physics engine!
            drift::ekfUpdateCamera(meas_vx, meas_vy, current_yaw, confidence); 
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
