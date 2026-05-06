/**
 * @file    transport.cpp
 * @brief   WiFi UDP transport implementation for Node 1
 *
 * @author  Sam (Firmware)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "transport.h"
#include "ekf.h"

static const char* WIFI_SSID = "esp32";
static const char* WIFI_PASSWORD = "esp32s3seed";
static const char* UDP_TARGET_IP = "255.255.255.255";
constexpr uint16_t UDP_TARGET_PORT = 4210;
constexpr uint16_t UDP_LOCAL_PORT = 4211;

static WiFiUDP udp;

bool drift::transportInit() {
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
        Serial.println("[NODE 1] WiFi connect failed.");
        return false;
    }

    Serial.printf("[NODE %d] WiFi connected, IP=%s\n", NODE_ID, WiFi.localIP().toString().c_str());
    udp.begin(UDP_LOCAL_PORT);
    Serial.printf("[NODE %d] UDP transport ready (remote %s:%u)\n", NODE_ID, UDP_TARGET_IP, UDP_TARGET_PORT);
    return true;
}

void drift::sendOdometryPacket(uint32_t timestamp_ms) {
    if (WiFi.status() != WL_CONNECTED) return;

    float px, py, yaw;
    drift::ekfGetPosition(px, py);
    drift::ekfGetOrientation(yaw);

    JsonDocument packet;
    packet["node"] = NODE_ID;
    packet["ts"] = timestamp_ms;
    packet["px"] = px;
    packet["py"] = py;
    packet["yaw"] = yaw;

    char buffer[256];
    size_t len = serializeJson(packet, buffer, sizeof(buffer));
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(buffer), len);
    udp.endPacket();
}