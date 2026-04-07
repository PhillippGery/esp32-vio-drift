/**
 * @file    main.cpp
 * @brief   PROJECT DRIFT — Node 1 VIO Firmware
 *          ESP32 Feather V2 + MPU-6050 + ArduCAM Mini SPI
 *
 * Pipeline overview:
 *  1. Read IMU at 200 Hz → pre-filter (complementary / Madgwick)
 *  2. Capture ArduCAM frame on trigger (feature extraction later)
 *  3. Run Extended Kalman Filter: predict (IMU) + update (camera features)
 *  4. Transmit JSON odometry packet over WiFi UDP at 50 Hz
 *
 * @author  Phillipp Gery (Kalman), Panchtio (Camera), Sam (Firmware)
 */


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include "ekf.h"
#include "status_led.h"
#include "vio_camera.h"
#include "mpu6050.h"

// TODO (Panchtio): include ArduCAM headers
// TODO (Phillipp): include EKF header from include/ekf.h


// ── I2C Pins for XIAO ESP32S3 ─────────────────────────────────────────────
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 5  
#endif
#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 6
#endif


// ── Task periods (ms) ─────────────────────────────────────────────────────
constexpr uint32_t IMU_PERIOD_MS   = 5;   // 200 Hz
constexpr uint32_t TX_PERIOD_MS    = 20;  // 50 Hz
constexpr uint32_t CAM_PERIOD_MS   = 200; // 5 Hz (placeholder)
constexpr uint32_t PRINT_PERIOD_MS = 200; // 5 Hz telemetry print

// ── Timing trackers ───────────────────────────────────────────────────────
static uint32_t lastImuMs  = 0;
static uint32_t lastTxMs   = 0;
static uint32_t lastCamMs  = 0;
static uint32_t lastPrintMs = 0;
MPU6050 imu;


constexpr bool VISION_ENABLED = true; 
float initial_yaw = 0.0f;

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.printf("[NODE %d] PROJECT DRIFT — VIO Node booting...\n", NODE_ID);

    // init status LED
    drift::ledInit();

    // Initialize I2C for the MPU-6050
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL);
    imu.begin(); // Default I2C address 0x68, can be changed if needed
    // Calibrate IMU (stationary, flat surface)
    // Init imu
    Serial.println("Calibrating — keep sensor still...");
    imu.calibrate();
    Serial.printf("=== Calibration Complete ===\n");
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n",imu.offsetAx, imu.offsetAy, imu.offsetAz);
    
    
    


    if (VISION_ENABLED) {
        // TODO (Panchtio): ArduCAM init + test JPEG capture
        // Initialize XIAO OV3660 Camera Pipeline & PSRAM
        if (drift::cameraInit()) {
            Serial.println("[NODE 1] Camera initialized successfully.");
        } else {
            Serial.println("[NODE 1] CRITICAL: Camera init failed! Halting.");
            while (true) { delay(1000); }
        }
    } else {
        Serial.println("[NODE 1] Camera disabled. Running IMU-only EKF.");
    }

    // TODO (Sam):    WiFi + UDP socket init


    // TODO (Phillipp): EKF init
    // Initialize Kalman Filter Matrices
    drift::ekfInit();
    delay(200); 
    drift::ekfGetOrientation(initial_yaw);
    Serial.println("[NODE 1] EKF Initialized.");
    drift::ekfReset(); 

    // Synchronize the clocks before starting the loop
    uint32_t start_time = millis();
    lastImuMs   = start_time;
    lastCamMs   = start_time;
    lastPrintMs = start_time;
    
    Serial.println("[NODE 1] Setup complete.");
    drift::ledSet(true);
}

// ------------------------------------------------------------
// loop() — full VIO pipeline with flow accumulation
// ------------------------------------------------------------
void loop() {
    uint32_t now = millis();


    // ── IMU read ─────────────────────────────────────────────────────────
    if (now - lastImuMs >= IMU_PERIOD_MS) {
        float dt = (now - lastImuMs) / 1000.0f; // dt in seconds
        lastImuMs = now;

        float ax, ay, az, gx, gy, gz;
        
        // read() automatically applies the static calibration offsets!
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            //Serial.printf("[NODE 1] IMU read: ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n", ax, ay, az, gx, gy, gz);
            
            // CRITICAL: Convert gyroscope Z from degrees/s to radians/s
            float gz_rad = gz * (PI / 180.0f);
            
            // Drive the physics engine forward
            drift::ekfPredict(ax, ay, gz_rad, dt);
        }
    }


    // ── Camera capture ───────────────────────────────────────────────────
    if (VISION_ENABLED && now - lastCamMs >= CAM_PERIOD_MS) {
        lastCamMs = now;
        // TODO (Panchtio): captureFrame() → feature extraction → EKF update

        float dx, dy, confidence;
        // If the optical flow accumulator finishes a window:
        // if (drift::cameraProcessFrame(dx, dy, confidence)) {
        //     // EKF UPDATE: Feed the visual displacement to the math engine!
        //     printf("[NODE 1] EKF updated with camera measurement: dx=%.3f m, dy=%.3f m, confidence=%.2f\n", dx, dy, confidence);
        // }


        // Klaman update with dummy camera data (static test)
        drift::ekfUpdateCamera(0.00f, 0.00f, initial_yaw, 0.95f);
    }

    // ── Debug triggers (Catch 'c' commands from Python) ──────────────────
    //drift::cameraDebugCheck();

    // ── Telemetry transmit ───────────────────────────────────────────────
    if (now - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = now;
        // TODO (Sam): serialize EKF state to JSON → UDP send
    }

    // ── 3. DEBUG EKF (5 Hz) ────────────────────────────────────────
    if (now - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = now;
        
        float px, py, yaw;
        drift::ekfGetPosition(px, py);
        drift::ekfGetOrientation(yaw);

        // Convert yaw back to degrees for easier 
        float yaw_deg = yaw * (180.0f / PI);

        // Print the EKF State
        Serial.printf("[EKF] X: %8.3f m | Y: %8.3f m | Yaw: %8.3f deg\n", px, py, yaw_deg);
    }
}
