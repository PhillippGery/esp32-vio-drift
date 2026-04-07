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

void setup() {
    Serial.begin(115200);
    Wire.begin(22, 14);
    imu.begin();

    drift::ledInit();
    drift::ledSet(drift::StatusColor::RED);
    
    

    // TODO (Sam): MPU-6050 init
    // TODO (Sam): WiFi + UDP socket init

    
    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
<<<<<<< HEAD
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n",
        imu.offsetAx, imu.offsetAy, imu.offsetAz);
=======
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);
    Serial.printf("[NODE %d] Setup complete.\n", NODE_ID);
    drift::ledSet(drift::StatusColor::GREEN);
>>>>>>> feature/node2-5-imu
}

void loop() {
    uint32_t now = millis();




    if (now - lastImuMs >= IMU_PERIOD_MS) {
        float dt = (now - lastImuMs) / 1000.0f; // dt in seconds
        lastImuMs = now;


        // TODO (Vedant): read IMU → store sample
        float ax, ay, az, gx, gy, gz;
        imu.read(ax, ay, az, gx, gy, gz);
        //Serial.printf("A: %.3f  %.3f  %.3f  |  G: %.3f  %.3f  %.3f\n", ax, ay, az, gx, gy, gz);
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            
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
        // TODO (Sam): serialize to JSON → UDP send to Node 1
    }
}
