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
#include <ArduinoJson.h>

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

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.printf("[NODE %d] PROJECT DRIFT — IMU Node booting...\n", NODE_ID);

    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);

    // TODO (Sam): MPU-6050 init
    // TODO (Sam): WiFi + UDP socket init

    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
}

void loop() {
    uint32_t now = millis();

    if (now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs = now;
        // TODO (Sam): read IMU → store sample
    }

    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        // TODO (Sam): serialize to JSON → UDP send to Node 1
    }
}
