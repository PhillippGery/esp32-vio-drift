/**
 * @file    main.cpp
 * @brief   PROJECT DRIFT — Node 2 IMU Reference Firmware
 *          ESP32 Feather V2 + MPU-6050
 *
 * Pipeline:
 *  1. Read MPU-6050 at 200 Hz
 *  2. Transmit JSON sensor packet over WiFi UDP at 100 Hz
 *
 * @author  Sam (Firmware)
 */

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "dead_reckoning.h"
#include "status_led.h"
#include "transport.h"

#ifndef NODE_ID
#define NODE_ID 2
#endif
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 22
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 20
#endif

constexpr uint32_t IMU_PERIOD_MS = 5;   // 200 Hz
constexpr uint32_t TX_PERIOD_MS  = 10;  // 100 Hz

static uint32_t lastImuMs = 0;
static uint32_t lastTxMs  = 0;

MPU6050 imu;
static drift::DeadReckoner tracker;
static float latestAx = 0.0f;
static float latestAy = 0.0f;
static float latestAz = 0.0f;
static float latestGx = 0.0f;
static float latestGy = 0.0f;
static float latestGz = 0.0f;

void setup() {
    Serial.begin(115200);
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    imu.begin();

    drift::ledInit();
    drift::ledSet(drift::StatusColor::RED);

    if (!drift::transportInit()) {
        Serial.println("[NODE 2] CRITICAL: WiFi init failed! Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);
    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
    drift::ledSet(drift::StatusColor::GREEN);
}

void loop() {
    uint32_t now = millis();

    if (now - lastImuMs >= IMU_PERIOD_MS) {
        uint32_t dt_ms = lastImuMs == 0 ? IMU_PERIOD_MS : (now - lastImuMs);
        lastImuMs = now;
        float dt = dt_ms / 1000.0f; // dt in seconds

        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            latestAx = ax;
            latestAy = ay;
            latestAz = az;
            latestGx = gx;
            latestGy = gy;
            latestGz = gz;

            float gz_rad = gz * (PI / 180.0f); // deg/s to rad/s
            tracker.update(ax, ay, gz_rad, dt);
        }
    }

    static uint32_t lastPrintMs = 0;
    if (now - lastPrintMs >= 500) {
        lastPrintMs = now;

        float total_drift = tracker.getDeltaMovement();
        Serial.printf("Pos: X:%.3f Y:%.3f | Total Delta: %.3f meters | Yaw: %.3f rad\n",
                      tracker.px, tracker.py, total_drift, tracker.yaw);
    }

    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        drift::sendImuPacket(now, latestAx, latestAy, latestAz, latestGx, latestGy, latestGz);
    }
}
