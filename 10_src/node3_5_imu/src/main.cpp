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
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <MPU6050.h>

#ifndef NODE_ID
#error "NODE_ID must be defined via platformio.ini build_flags (-DNODE_ID=3)"
#endif
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 22
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 20
#endif

constexpr uint32_t IMU_PERIOD_MS = 5;
constexpr uint32_t TX_PERIOD_MS  = 10;

static uint32_t lastImuMs = 0;
static uint32_t lastTxMs  = 0;

MPU6050 imu;
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
constexpr uint16_t UDP_LOCAL_PORT = 4213;

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
        Serial.printf("[NODE %d] WiFi connect failed.\n", NODE_ID);
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
    while (!Serial && millis() < 3000) {}

    Serial.printf("[NODE %d] PROJECT DRIFT — IMU Node booting...\n", NODE_ID);

    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    imu.begin();

    if (!startWiFi()) {
        Serial.printf("[NODE %d] CRITICAL: WiFi init failed! Halting.\n", NODE_ID);
        while (true) { delay(1000); }
    }
    udp.begin(UDP_LOCAL_PORT);
    Serial.printf("[NODE %d] UDP transport ready (remote %s:%u)\n", NODE_ID, UDP_TARGET_IP, UDP_TARGET_PORT);

    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);
    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
}


void loop() {
    uint32_t now = millis();

    if (now - lastImuMs >= IMU_PERIOD_MS) {
        lastImuMs = now;
        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            latestAx = ax;
            latestAy = ay;
            latestAz = az;
            latestGx = gx;
            latestGy = gy;
            latestGz = gz;
        }
    }

    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        sendImuPacket(now);
    }
}
