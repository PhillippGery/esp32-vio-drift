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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

constexpr uint32_t IMU_PERIOD_MS  = 5;   // 200 Hz
constexpr uint32_t TX_PERIOD_MS   = 10;  // 100 Hz
constexpr uint32_t PRINT_PERIOD_MS = 500;
constexpr bool WIFI_ENABLED  = true;
constexpr bool DEBUG_PRINT   = true;

// ── ImuData: passed between cores via imuQueue ────────────────────────────
struct ImuData {
    float ax, ay, az;
    float gx, gy, gz; // gz in deg/s (raw, converted before tracker update)
    float dt;
};

static QueueHandle_t imuQueue;

MPU6050 imu;
static drift::DeadReckoner tracker;

// Latest raw values — written by fusionTask, read by telemetryTask (same core)
static float latestAx = 0.0f, latestAy = 0.0f, latestAz = 0.0f;
static float latestGx = 0.0f, latestGy = 0.0f, latestGz = 0.0f;


// ─────────────────────────────────────────────────────────────────────────
// Core 0 — imuTask   Priority 5   200 Hz
// CRITICAL: No xSemaphoreTake / Mutex — this task must never block.
// ─────────────────────────────────────────────────────────────────────────
void imuTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(IMU_PERIOD_MS);
    uint32_t lastReadMs = millis();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        uint32_t now = millis();
        float dt = (now - lastReadMs) / 1000.0f;
        lastReadMs = now;

        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            ImuData data = { ax, ay, az, gx, gy, gz, dt };
            xQueueSend(imuQueue, &data, 0); // non-blocking: drop if full
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────
// Core 1 — fusionTask   Priority 4
// Receives IMU data, updates dead reckoner, caches latest raw values.
// ─────────────────────────────────────────────────────────────────────────
void fusionTask(void* pvParameters) {
    uint32_t lastPrintMs = millis();

    for (;;) {
        ImuData data;
        xQueueReceive(imuQueue, &data, portMAX_DELAY);

        float gz_rad = data.gz * (PI / 180.0f);
        tracker.update(data.ax, data.ay, gz_rad, data.dt);

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
    imu.begin();

    drift::ledInit();
    drift::ledSet(drift::StatusColor::RED);

    if (WIFI_ENABLED) {
        if (!drift::transportInit()) {
            Serial.println("[NODE 2] CRITICAL: WiFi init failed! Halting.");
            while (true) { delay(1000); }
        }
    } else {
        Serial.println("[NODE 2] WiFi disabled. Running offline.");
    }

    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);

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
