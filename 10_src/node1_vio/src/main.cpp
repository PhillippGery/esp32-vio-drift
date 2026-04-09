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
#include "ekf.h"
#include "status_led.h"
#include "vio_camera.h"
#include "MPU6050.h"
#include "transport.h"

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



    if (drift::cameraInit()) {
        Serial.println("[NODE 1] Camera initialized successfully.");
    } else {
        Serial.println("[NODE 1] CRITICAL: Camera init failed! Halting.");
        while (true) { delay(1000); }
    }

    if (!drift::transportInit()) {
        Serial.println("[NODE 1] CRITICAL: WiFi init failed! Halting.");
        while (true) { delay(1000); }
    }


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

    // ── Telemetry TX ────────────────────────────────────────────────────────
    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        drift::sendOdometryPacket(now);
    }
}

