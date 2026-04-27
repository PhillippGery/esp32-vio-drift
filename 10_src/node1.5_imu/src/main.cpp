/**
 * @file    main.cpp
 * @brief   PROJECT DRIFT — Node 2 IMU Reference Firmware
 * ESP32 Feather V2 + Würth ISDS
 *
 * Pipeline:
 * 1. Read ISDS continuously at hardware speed (208 Hz)
 * 2. Transmit JSON sensor packet over WiFi UDP at 100 Hz
 *
 * @author  Sam (Firmware)
 */

#include <Arduino.h>
#include <Wire.h>
#include "WurthISDS.h" // <-- 1. Swapped the wrapper
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
#define IMU_I2C_SCL 20 // Assuming you moved the wire back to 20!
#endif

constexpr uint32_t TX_PERIOD_MS  = 10;  // 100 Hz

static uint32_t lastImuUs = 0; // <-- Changed to Microseconds
static uint32_t lastTxMs  = 0;

WurthISDS imu; // <-- 1. Swapped the object
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
    
    drift::ledInit();
    drift::ledSet(drift::StatusColor::RED);

    // Added a safety check just in case!
    if (!imu.begin()) {
        Serial.println("[NODE 2] CRITICAL: ISDS IMU init failed! Halting.");
        while (true) { delay(1000); }
    }

    if (!drift::transportInit()) {
        Serial.println("[NODE 2] CRITICAL: WiFi init failed! Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);
    
    lastImuUs = micros(); // Start the high-res timer
    
    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
    drift::ledSet(drift::StatusColor::GREEN);
}

void loop() {
    uint32_t now_ms = millis();
    uint32_t now_us = micros();

    float ax, ay, az, gx, gy, gz;
    
    // 3. Removed artificial 5ms timer. Let the sensor govern the 208Hz speed!
    if (imu.read(ax, ay, az, gx, gy, gz)) {
        
        float dt = (now_us - lastImuUs) / 1000000.0f; // dt in seconds
        lastImuUs = now_us;

        latestAx = ax;
        latestAy = ay;
        latestAz = az;
        latestGx = gx;
        latestGy = gy;
        latestGz = gz;

        // 2. Removed the PI/180 conversion. gz is already in rad/s!
        tracker.update(ax, ay, gz, dt); 
    }

    static uint32_t lastPrintMs = 0;
    if (now_ms - lastPrintMs >= 500) {
        lastPrintMs = now_ms;

        float total_drift = tracker.getDeltaMovement();
        Serial.printf("Pos: X:%.3f Y:%.3f | Total Delta: %.3f meters | Yaw: %.3f rad\n",
                      tracker.px, tracker.py, total_drift, tracker.yaw);
    }

    if (now_ms - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now_ms;
        drift::sendImuPacket(now_ms, latestAx, latestAy, latestAz, latestGx, latestGy, latestGz);
    }
}