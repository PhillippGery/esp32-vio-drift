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
 * @course  ECE 56800 — Purdue University, Spring 2026
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>

// TODO (Panchtio): include ArduCAM headers
// TODO (Phillipp): include EKF header from include/ekf.h

// ── Pin Definitions (also set via build_flags in platformio.ini) ──────────
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 22
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 20
#endif
#ifndef CAM_SPI_CS
#define CAM_SPI_CS 33
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

    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    SPI.begin();

    // TODO (Sam):    WiFi + UDP socket init
    // TODO (Phillipp): EKF init
    // TODO (Panchtio): ArduCAM init + test JPEG capture

    Serial.println("[NODE 1] Setup complete.");
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // ── IMU read ─────────────────────────────────────────────────────────
    if (now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs = now;
        // TODO (Phillipp): readIMU() → EKF predict step
    }

    // ── Camera capture ───────────────────────────────────────────────────
    if (now - lastCamMs >= CAM_PERIOD_MS) {
        lastCamMs = now;
        // TODO (Panchtio): captureFrame() → feature extraction → EKF update
    }

    // ── Telemetry transmit ───────────────────────────────────────────────
    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        // TODO (Sam): serialize EKF state to JSON → UDP send
    }
}
