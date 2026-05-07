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
#include <Arduino.h>

static const char* WIFI_SSID = "Vpat";
static const char* WIFI_PASSWORD = "12345678";
static const char* UDP_TARGET_IP = "10.42.0.1";
constexpr uint16_t UDP_TARGET_PORT = 4210;
constexpr uint16_t UDP_LOCAL_PORT = 4211;

static WiFiUDP udp;

bool drift::transportInit() {
    Serial.printf("[NODE %d] Connecting to WiFi SSID='%s'...\n", NODE_ID, WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm); // Reduce transmit power to save energy and reduce interference
    WiFi.setSleep(false); // Disable WiFi modem sleep to reduce latency and improve stability for UDP transport
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
    float temperature_c = temperatureRead();

    JsonDocument packet;
    packet["node"] = NODE_ID;
    packet["ts"] = timestamp_ms;
    packet["px"] = px;
    packet["py"] = py;
    packet["yaw"] = yaw;
    packet["temperature_c"] = temperature_c;

    char buffer[256];
    size_t len = serializeJson(packet, buffer, sizeof(buffer));
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(buffer), len);
    udp.endPacket();
}