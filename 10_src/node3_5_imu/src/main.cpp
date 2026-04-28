/**
 * @file    main.cpp
 * @brief   PROJECT DRIFT — Nodes 3–5 Shared IMU Firmware
 *          ESP32 Feather V2 + MPU-6050 (identical hardware to Node 2)
 *
 *  NODE_ID is injected at compile time via platformio.ini build_flags.
 *  Flash with: pio run -e node3 --target upload   (or node4 / node5)
 *
 * @author  Sam (Firmware)
 */

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"
#include "transport.h"

#ifndef NODE_ID
#error "NODE_ID must be defined via platformio.ini build_flags (-DNODE_ID=3)"
#endif
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 22
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 20
#endif

constexpr uint32_t IMU_PERIOD_MS = 5;   // 200 Hz
constexpr uint32_t TX_PERIOD_MS  = 10;  // 100 Hz
constexpr bool WIFI_ENABLED = true;

// Latest raw IMU values — written by imuTask (Core 0), read by telemetryTask (Core 1).
// Single-writer / single-reader with no ordering dependency: stale-by-one-frame is acceptable.
static volatile float latestAx = 0.0f, latestAy = 0.0f, latestAz = 0.0f;
static volatile float latestGx = 0.0f, latestGy = 0.0f, latestGz = 0.0f;

MPU6050 imu;


// ─────────────────────────────────────────────────────────────────────────
// Core 0 — imuTask   Priority 5   200 Hz
// CRITICAL: No xSemaphoreTake / Mutex — this task must never block.
// ─────────────────────────────────────────────────────────────────────────
void imuTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(IMU_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            latestAx = ax; latestAy = ay; latestAz = az;
            latestGx = gx; latestGy = gy; latestGz = gz;
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────
// Core 1 — telemetryTask   Priority 2   100 Hz
// ─────────────────────────────────────────────────────────────────────────
void telemetryTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TX_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        drift::sendImuPacket(millis(),
                             latestAx, latestAy, latestAz,
                             latestGx, latestGy, latestGz);
    }
}


// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.printf("[NODE %d] PROJECT DRIFT — IMU Node booting...\n", NODE_ID);

    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    imu.begin();

    if (WIFI_ENABLED) {
        if (!drift::transportInit()) {
            Serial.printf("[NODE %d] CRITICAL: WiFi init failed! Halting.\n", NODE_ID);
            while (true) { delay(1000); }
        }
    } else {
        Serial.printf("[NODE %d] WiFi disabled. Running offline.\n", NODE_ID);
    }

    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);

    xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, nullptr, 5, nullptr, 0);
    if (WIFI_ENABLED) {
        xTaskCreatePinnedToCore(telemetryTask, "telemetryTask", 4096, nullptr, 2, nullptr, 1);
    }

    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
}

void loop() {
    vTaskDelete(nullptr);
}
