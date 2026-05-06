/**
 * @file    transport.cpp
 * @brief   WiFi UDP transport implementation for Node 2
 *
 * @author  Sam (Firmware)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <array>
#include "transport.h"

static const char* WIFI_SSID = "Jack's Surface";
static const char* WIFI_PASSWORD = "Hello World!";
static const char* UDP_TARGET_IP = "255.255.255.255";
constexpr uint16_t UDP_TARGET_PORT = 4210;
constexpr uint16_t UDP_LOCAL_PORT = 4212;

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
        Serial.println("[NODE 2] WiFi connect failed.");
        return false;
    }

    Serial.printf("[NODE %d] WiFi connected, IP=%s\n", NODE_ID, WiFi.localIP().toString().c_str());
    udp.begin(UDP_LOCAL_PORT);
    Serial.printf("[NODE %d] UDP transport ready (remote %s:%u)\n", NODE_ID, UDP_TARGET_IP, UDP_TARGET_PORT);
    return true;
}


void drift::sendImuPacket(uint32_t timestamp_ms, float ax, float ay, float az, float gx, float gy, float gz, float temp_c) {
    StaticJsonDocument<224> packet;
    packet["node"] = NODE_ID;
    packet["ts"] = timestamp_ms;
    packet["ax"] = ax;
    packet["ay"] = ay;
    packet["az"] = az;
    packet["gx"] = gx;
    packet["gy"] = gy;
    packet["gz"] = gz;
    packet["temp_c"] = temp_c;

    char buffer[256];
    size_t len = serializeJson(packet, buffer, sizeof(buffer));
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(buffer), len);
    udp.endPacket();
}