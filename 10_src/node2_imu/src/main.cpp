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
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "MPU6050.h"
#include "dead_reckoning.h"
#include "status_led.h"

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
static WiFiUDP udp;

static float latestAx = 0.0f;
static float latestAy = 0.0f;
static float latestAz = 0.0f;
static float latestGx = 0.0f;
static float latestGy = 0.0f;
static float latestGz = 0.0f;

static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
static const char* UDP_TARGET_IP = "255.255.255.255";
constexpr uint16_t UDP_TARGET_PORT = 4210;
constexpr uint16_t UDP_LOCAL_PORT = 4212;

static bool startWiFi() {
    Serial.printf("[NODE %d] Connecting to WiFi SSID='%s'...\n", NODE_ID, WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t timeout_ms = 15000;
    uint32_t start_ms = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_ms < timeout_ms) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[NODE 2] WiFi connect failed.");
        return false;
    }

    Serial.printf("[NODE %d] WiFi connected, IP=%s\n", NODE_ID, WiFi.localIP().toString().c_str());
    return true;
}

static void sendImuPacket(uint32_t timestamp_ms) {
    StaticJsonDocument<192> packet;
    packet["node"] = NODE_ID;
    packet["ts"] = timestamp_ms;
    packet["ax"] = latestAx;
    packet["ay"] = latestAy;
    packet["az"] = latestAz;
    packet["gx"] = latestGx;
    packet["gy"] = latestGy;
    packet["gz"] = latestGz;

    char buffer[256];
    size_t len = serializeJson(packet, buffer, sizeof(buffer));
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(buffer), len);
    udp.endPacket();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    imu.begin();

    drift::ledInit();
    drift::ledSet(drift::StatusColor::RED);
    
    if (!startWiFi()) {
        Serial.println("[NODE 2] CRITICAL: WiFi init failed! Halting.");
        while (true) { delay(1000); }
    }
    udp.begin(UDP_LOCAL_PORT);
    Serial.printf("[NODE %d] UDP transport ready (remote %s:%u)\n", NODE_ID, UDP_TARGET_IP, UDP_TARGET_PORT);

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
        
        // 5. Get the total delta movement
        float total_drift = tracker.getDeltaMovement();
        
        Serial.printf("Pos: X:%.3f Y:%.3f | Total Delta: %.3f meters\n | Yaw: %.3f rad", tracker.px, tracker.py, total_drift, tracker.yaw);
    }


    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        sendImuPacket(now);
    }
}
