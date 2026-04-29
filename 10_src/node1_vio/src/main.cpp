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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ekf.h"
#include "status_led.h"
#include "vio_camera.h"
//#include "MPU6050.h"
#include "transport.h"
#include "WurthISDS.h"

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
constexpr uint32_t CAM_PERIOD_MS   = 50; // 20 Hz (placeholder)
constexpr uint32_t PRINT_PERIOD_MS = 300; // 5 Hz telemetry print
constexpr bool VISION_ENABLED = true;  // Set to false to disable camera and run IMU-only EKF
constexpr bool WIFI_ENABLED   = false;  // Set to false to disable WiFi/UDP transport
constexpr bool DEBUG_PRINT    = true;  // Set to false to suppress EKF position serial output

// ── ImuData: passed between cores via imuQueue ────────────────────────────
struct ImuData {
    float ax;
    float ay;
    float gz; // rad/s
    float dt; // seconds
};

// ── FreeRTOS inter-core queue ─────────────────────────────────────────────
static QueueHandle_t imuQueue;

// ── Shared state (camera fusion, read/written only from fusionTask) ────────
static float initial_yaw       = 0.0f;
static float last_cam_yaw      = initial_yaw;
static float last_cam_confidence = 0.0f;

WurthISDS imu;


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

        // read() automatically applies the static calibration offsets!
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            // CRITICAL: Convert gyroscope Z from degrees/s to radians/s
            //float gz_rad = gz * (PI / 180.0f);

            ImuData data = { ax, ay, gz, dt };
            xQueueSend(imuQueue, &data, 0); // non-blocking: drop if full
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────
// Core 1 — fusionTask   Priority 4
// Blocks on imuQueue, then runs EKF predict + optional camera update.
// ─────────────────────────────────────────────────────────────────────────
void fusionTask(void* pvParameters) {
    uint32_t lastCamMs        = millis(); // rate-limit: reset AFTER camera returns
    uint32_t lastCamCaptureMs = millis(); // true inter-frame dt for velocity computation
    uint32_t lastPrintMs      = millis();

    for (;;) {
        ImuData data;
        xQueueReceive(imuQueue, &data, portMAX_DELAY);

        // Drive the physics engine forward
        drift::ekfPredict(data.ax, data.ay, data.gz, data.dt);

        // ── ZUPT: Zero-velocity / Zero-rate update ───────────────────────────
        // Debounced: must be continuously stationary for 300 ms before firing.
        // Without the debounce, ZUPT fires during the rotation ramp-down (gz still
        // passing through the threshold), injecting a bgz value that contains real
        // rotation. The negative P[yaw,bgz] cross-covariance then pulls yaw down,
        // eating into the correctly integrated angle (observed as 70–80° for 90° turns).
        {
            static uint32_t stillSinceMs = 0;
            static bool wasStill = false;
            float acc_h = sqrtf(data.ax * data.ax + data.ay * data.ay);
            bool isStill = (acc_h < 0.3f) && (fabsf(data.gz) < 0.008f);
            if (isStill) {
                if (!wasStill) { stillSinceMs = millis(); wasStill = true; }
                if ((millis() - stillSinceMs) > 300) {
                    drift::ekfZupt(data.gz);
                }
            } else {
                wasStill = false;
            }
        }

        uint32_t now = millis();

        // ── Camera capture ───────────────────────────────────────────────────
        if (VISION_ENABLED && (now - lastCamMs >= CAM_PERIOD_MS)) {
            // cam_dt = true time between frame grabs, not time between checks.
            // lastCamMs is reset AFTER the call returns (see below), so the
            // camera cannot re-fire until CAM_PERIOD_MS has elapsed since the
            // previous call finished — preventing back-to-back fires that give
            // delta_yaw_imu only one IMU step of rotation for an 80ms frame gap.
            float cam_dt = (now - lastCamCaptureMs) / 1000.0f;
            lastCamCaptureMs = now;

            // 1. Calculate how much the IMU rotated since the last camera frame
            float current_yaw;
            drift::ekfGetOrientation(current_yaw);

            // Calculate shortest path angular difference to prevent 360-degree wrap-around bugs
            float delta_yaw_imu = current_yaw - last_cam_yaw;
            while (delta_yaw_imu > M_PI)  delta_yaw_imu -= 2.0f * M_PI;
            while (delta_yaw_imu < -M_PI) delta_yaw_imu += 2.0f * M_PI;

            last_cam_yaw = current_yaw; // Save for next frame

            float meas_vx, meas_vy, confidence;

            // 2. Pass the dt AND the rotation fix into the camera processor
            if (drift::cameraProcessFrame(meas_vx, meas_vy, confidence, cam_dt, delta_yaw_imu)) {
                last_cam_confidence = confidence;
                // Gate: skip camera update during fast rotation. Imperfect de-rotation
                // produces a spurious translational velocity that, through the yaw↔vx
                // cross-covariance, pulls the yaw estimate down. 0.2 rad/s ≈ 11 deg/s.
                bool spinning = fabsf(delta_yaw_imu / cam_dt) > 0.2f;
                if (!spinning) {
                    drift::ekfUpdateCamera(meas_vx, meas_vy, current_yaw, confidence);
                }
            }

            // Reset rate-limit AFTER camera returns so we wait CAM_PERIOD_MS
            // from the END of this call, not the start.
            lastCamMs = millis();
        }

        if (DEBUG_PRINT && (now - lastPrintMs >= PRINT_PERIOD_MS)) {
            lastPrintMs = now;

            float px, py, yaw;
            drift::ekfGetPosition(px, py);
            drift::ekfGetOrientation(yaw);
            float yaw_deg = yaw * (180.0f / PI);

            float bgz;
            drift::ekfGetBias(bgz);
            float bgz_mdps = bgz * (180.0f / PI) * 1000.0f;
            Serial.printf("[EKF] X:%8.3f m  Y:%8.3f m  Yaw:%8.3f deg  bgz:%+.2f mdps  conf:%.2f\n",
                          px, py, yaw_deg, bgz_mdps, last_cam_confidence);
        }

    }
}


// ─────────────────────────────────────────────────────────────────────────
// Core 1 — telemetryTask   Priority 2   50 Hz
// ─────────────────────────────────────────────────────────────────────────
void telemetryTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TX_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        drift::sendOdometryPacket(millis());
    }
}


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
    Serial.printf("Offsets ax:%.4f ay:%.4f az:%.4f\n", imu.offsetAx, imu.offsetAy, imu.offsetAz);
    Serial.printf("Gyro offsets gx:%.4f gy:%.4f gz:%.4f mdps\n", imu.offsetGx, imu.offsetGy, imu.offsetGz);



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

    if (WIFI_ENABLED) {
        if (!drift::transportInit()) {
            Serial.println("[NODE 1] CRITICAL: WiFi/UDP init failed! Halting.");
            while (true) { delay(1000); }
        }
        Serial.println("[NODE 1] Transport initialized.");
    } else {
        Serial.println("[NODE 1] WiFi disabled. Running offline.");
    }


    // TODO (Phillipp): EKF init
// Initialize Kalman Filter Matrices
    drift::ekfInit();
    delay(200);
    drift::ekfGetOrientation(initial_yaw);
    Serial.println("[NODE 1] EKF Initialized.");
    drift::ekfReset();

    // Create the inter-core queue before spawning tasks
    imuQueue = xQueueCreate(20, sizeof(ImuData));

    // Spawn tasks — imuTask on Core 0, fusion + telemetry on Core 1
    xTaskCreatePinnedToCore(imuTask,    "imuTask",    4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(fusionTask, "fusionTask", 8192, nullptr, 4, nullptr, 1);
    if (WIFI_ENABLED) {
        xTaskCreatePinnedToCore(telemetryTask, "telemetryTask", 4096, nullptr, 2, nullptr, 1);
    }

    Serial.println("[NODE 1] Setup complete.");
    drift::ledSet(true);
}

// ------------------------------------------------------------
// loop() — surrendered to FreeRTOS; all work is in pinned tasks
// ------------------------------------------------------------
void loop() {
    vTaskDelete(nullptr);
}
