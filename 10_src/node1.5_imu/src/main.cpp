/**
 * @file    main.cpp
 * @brief   PROJECT DRIFT — Node 1.5 IMU Firmware
 *          ESP32 + Würth ISDS (208 Hz hardware rate)
 *
 * Pipeline:
 *  1. Poll ISDS at hardware rate (~208 Hz) — sensor gates actual reads
 *  2. Update dead reckoner on Core 1
 *  3. Transmit JSON sensor packet over WiFi UDP at 100 Hz
 *
 * @author  Sam (Firmware)
 */

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "WurthISDS.h"
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

// Poll faster than 208 Hz so imu.read() gates the true data-ready rate
constexpr uint32_t IMU_POLL_MS   = 4;   // 250 Hz poll — sensor gates to ~208 Hz
constexpr uint32_t TX_PERIOD_MS  = 10;  // 100 Hz
constexpr uint32_t PRINT_PERIOD_MS = 500;
constexpr bool WIFI_ENABLED  = true;
constexpr bool DEBUG_PRINT   = true;

// ── ImuData: passed between cores via imuQueue ────────────────────────────
struct ImuData {
    float ax, ay, az;
    float gx, gy, gz; // gz already in rad/s from ISDS driver
    float dt;         // seconds — actual elapsed time between reads
};

static QueueHandle_t imuQueue;

WurthISDS imu;
static drift::DeadReckoner tracker;

// Latest raw values — written by fusionTask, read by telemetryTask (same core)
static float latestAx = 0.0f, latestAy = 0.0f, latestAz = 0.0f;
static float latestGx = 0.0f, latestGy = 0.0f, latestGz = 0.0f;


// ─────────────────────────────────────────────────────────────────────────
// Core 0 — imuTask   Priority 5   ~208 Hz (sensor-gated)
// Polls at 250 Hz; imu.read() returns true only when new data is ready.
// CRITICAL: No xSemaphoreTake / Mutex — this task must never block.
// ─────────────────────────────────────────────────────────────────────────
void imuTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(IMU_POLL_MS);
    uint32_t lastReadUs = micros();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            uint32_t now_us = micros();
            float dt = (now_us - lastReadUs) / 1000000.0f;
            lastReadUs = now_us;

            ImuData data = { ax, ay, az, gx, gy, gz, dt };
            xQueueSend(imuQueue, &data, 0); // non-blocking: drop if full
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────
// Core 1 — fusionTask   Priority 4
// Receives IMU data, updates dead reckoner, caches latest raw values.
// gz from ISDS is already in rad/s — no conversion needed.
// ─────────────────────────────────────────────────────────────────────────
void fusionTask(void* pvParameters) {
    uint32_t lastPrintMs = millis();

    for (;;) {
        ImuData data;
        xQueueReceive(imuQueue, &data, portMAX_DELAY);

        tracker.update(data.ax, data.ay, data.gz, data.dt);

        latestAx = data.ax; latestAy = data.ay; latestAz = data.az;
        latestGx = data.gx; latestGy = data.gy; latestGz = data.gz;

        if (DEBUG_PRINT) {
            uint32_t now = millis();
            if (now - lastPrintMs >= PRINT_PERIOD_MS) {
                lastPrintMs = now;
                float total_drift = tracker.getDeltaMovement();
                float bgz_mdps = tracker.gz_bias * (180.0f / M_PI) * 1000.0f;
                Serial.printf("Pos: X:%.3f Y:%.3f | Delta: %.3f m | Yaw: %.3f rad | bias: %+.2f mdps\n",
                              tracker.px, tracker.py, total_drift, tracker.yaw, bgz_mdps);
            }
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
        drift::sendImuPacket(millis(), latestAx, latestAy, latestAz, latestGx, latestGy, latestGz);
    }
}


// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);

    drift::ledInit();
    drift::ledSet(drift::StatusColor::RED);

    if (!imu.begin()) {
        Serial.println("[NODE 1.5] CRITICAL: ISDS IMU init failed! Halting.");
        while (true) { delay(1000); }
    }

    if (WIFI_ENABLED) {
        if (!drift::transportInit()) {
            Serial.println("[NODE 1.5] CRITICAL: WiFi init failed! Halting.");
            while (true) { delay(1000); }
        }
    } else {
        Serial.println("[NODE 1.5] WiFi disabled. Running offline.");
    }

    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);
    Serial.printf("Gyro offsets gx:%.4f gy:%.4f gz:%.4f mdps\n", imu.offsetGx, imu.offsetGy, imu.offsetGz);

    imuQueue = xQueueCreate(20, sizeof(ImuData));

    xTaskCreatePinnedToCore(imuTask,    "imuTask",    4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(fusionTask, "fusionTask", 4096, nullptr, 4, nullptr, 1);
    if (WIFI_ENABLED) {
        xTaskCreatePinnedToCore(telemetryTask, "telemetryTask", 4096, nullptr, 2, nullptr, 1);
    }

    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
    drift::ledSet(drift::StatusColor::GREEN);
}

void loop() {
    vTaskDelete(nullptr);
}
