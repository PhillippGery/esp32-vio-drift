#ifndef WURTH_ISDS_H
#define WURTH_ISDS_H

#include <Arduino.h>
#include <Wire.h>

// Include the Würth C headers
extern "C" {
    #include "WSEN_ISDS_2536030320001.h"
}

template<int N = 5>
class MovingAvg {
    float buf[N] = {};
    int idx = 0;
    bool full = false;
public:
    float update(float val) {
        buf[idx] = val;
        idx = (idx + 1) % N;
        if (idx == 0) full = true;
        int count = full ? N : idx;
        float sum = 0;
        for (int i = 0; i < count; i++) sum += buf[i];
        return sum / count;
    }
};

class WurthISDS {
public:
    // Calibration offsets
    float offsetAx = 0.0f, offsetAy = 0.0f, offsetAz = 0.0f;
    float offsetGx = 0.0f, offsetGy = 0.0f, offsetGz = 0.0f;

    WE_sensorInterface_t sensorInterface;

    bool begin() {
        // Initialize the default sensor interface from the driver
        ISDS_getDefaultInterface(&sensorInterface);
        
        // Ensure standard I2C address is set (0x6B by default on the EV board)
        sensorInterface.options.i2c.address = ISDS_ADDRESS_I2C_1; // Note: I2C_1 is 0x6B, I2C_0 is 0x6A

        // Soft reset the sensor
        ISDS_softReset(&sensorInterface, ISDS_enable);
        delay(50); // Wait for boot

        // Set Full Scales and Output Data Rates using the SDK functions
        ISDS_setAccFullScale(&sensorInterface, ISDS_accFullScaleFourG);
        ISDS_setAccOutputDataRate(&sensorInterface, ISDS_accOdr208Hz);

        ISDS_setGyroFullScale(&sensorInterface, ISDS_gyroFullScale250dps);
        ISDS_setGyroOutputDataRate(&sensorInterface, ISDS_gyroOdr208Hz);
        ISDS_enableGyroDigitalLpf1(&sensorInterface, ISDS_enable);

        // Turn on Block Data Update (BDU) for stable reads
        ISDS_enableBlockDataUpdate(&sensorInterface, ISDS_enable);

        return true;
    }

    // --- CALIBRATION ROUTINE ---
    void calibrate(int samples = 500) {
        float sumAx = 0, sumAy = 0, sumAz = 0;
        float sumGx = 0, sumGy = 0, sumGz = 0;
        float ax, ay, az, gx, gy, gz;

        // Wait for the sensor to thermally stabilize before collecting bias samples
        delay(1000);
        for (int i = 0; i < 100; i++) {
            readRaw(ax, ay, az, gx, gy, gz);
            delay(5);
        }

        for (int i = 0; i < samples; i++) {
            readRaw(ax, ay, az, gx, gy, gz);
            sumAx += ax;
            sumAy += ay;
            sumAz += az;
            sumGx += gx;
            sumGy += gy;
            sumGz += gz;
            delay(5); // Wait roughly 1 period at 200Hz
        }

        // Calculate averages
        offsetAx = sumAx / samples;
        offsetAy = sumAy / samples;
        // Gravity is typically 1g (1000mg) on the Z axis assuming it is flat on a table
        offsetAz = (sumAz / samples) - 1000.0f; 
        
        offsetGx = sumGx / samples;
        offsetGy = sumGy / samples;
        offsetGz = sumGz / samples;
    }

    bool read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        // Check if data is ready using the driver's status register function
        ISDS_state_t accReady, gyroReady;
        ISDS_isDataReady(&sensorInterface, NULL, &accReady, &gyroReady);
        
        if (accReady == ISDS_enable && gyroReady == ISDS_enable) {
            readRaw(ax, ay, az, gx, gy, gz);
            
            // 1. Apply calibration offsets (calculated in mg and mdps)
            ax -= offsetAx;
            ay -= offsetAy;
            az -= offsetAz;
            gx -= offsetGx;
            gy -= offsetGy;
            gz -= offsetGz;
            
            // 2. Convert to m/s^2 (matching ekf.cpp gravity exactly)
            ax = (ax / 1000.0f) * 9.81f;
            ay = (ay / 1000.0f) * 9.81f;
            az = (az / 1000.0f) * 9.81f;

            gx = (gx / 1000.0f) * (PI / 180.0f);
            gy = (gy / 1000.0f) * (PI / 180.0f);
            gz = _fgz.update((gz / 1000.0f) * (PI / 180.0f));

            return true;
        }
        return false;
    }

private:
    MovingAvg<5> _fgz;

    void readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        // Use the driver's float conversion functions
        ISDS_getAccelerations_float(&sensorInterface, &ax, &ay, &az);
        ISDS_getAngularRates_float(&sensorInterface, &gx, &gy, &gz);
    }
};

// --- REQUIRED PLATFORM FUNCTIONS ---
// The Würth C SDK expects these functions to be implemented in your project 
// to handle the actual hardware I2C transmission.
extern "C" {
    int8_t WE_ReadReg(WE_sensorInterface_t* interface, uint8_t regAdr, uint16_t numBytesToRead, uint8_t* data) {
        Wire.beginTransmission(interface->options.i2c.address);
        Wire.write(regAdr);
        if (Wire.endTransmission(false) != 0) return WE_FAIL;
        
        Wire.requestFrom((uint8_t)interface->options.i2c.address, (uint8_t)numBytesToRead);
        for (uint16_t i = 0; i < numBytesToRead; i++) {
            if (Wire.available()) data[i] = Wire.read();
        }
        return WE_SUCCESS;
    }

    int8_t WE_WriteReg(WE_sensorInterface_t* interface, uint8_t regAdr, uint16_t numBytesToWrite, uint8_t* data) {
        Wire.beginTransmission(interface->options.i2c.address);
        Wire.write(regAdr);
        for (uint16_t i = 0; i < numBytesToWrite; i++) {
            Wire.write(data[i]);
        }
        return (Wire.endTransmission() == 0) ? WE_SUCCESS : WE_FAIL;
    }
}

#endif // WURTH_ISDS_H