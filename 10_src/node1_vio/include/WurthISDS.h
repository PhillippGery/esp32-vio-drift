#ifndef WURTH_ISDS_H
#define WURTH_ISDS_H

#include <Arduino.h>
#include <Wire.h>

// Include the Würth C headers
extern "C" {
    #include "WSEN_ISDS_2536030320001.h"
}

class WurthISDS {
public:
    float offsetAx = 0.0f, offsetAy = 0.0f, offsetAz = 0.0f;
    float offsetGx = 0.0f, offsetGy = 0.0f, offsetGz = 0.0f;

    WE_sensorInterface_t sensorInterface;

    bool begin() {
        ISDS_getDefaultInterface(&sensorInterface);
        sensorInterface.options.i2c.address = ISDS_ADDRESS_I2C_1; 

        ISDS_softReset(&sensorInterface, ISDS_enable);
        delay(50); // Wait for boot

        ISDS_setAccFullScale(&sensorInterface, ISDS_accFullScaleFourG);
        ISDS_setAccOutputDataRate(&sensorInterface, ISDS_accOdr208Hz);
        
        ISDS_setGyroFullScale(&sensorInterface, ISDS_gyroFullScale500dps);
        ISDS_setGyroOutputDataRate(&sensorInterface, ISDS_gyroOdr208Hz);

        ISDS_enableBlockDataUpdate(&sensorInterface, ISDS_enable);
        return true;
    }

    void calibrate(int samples = 500) {
        float sumAx = 0, sumAy = 0, sumAz = 0;
        float sumGx = 0, sumGy = 0, sumGz = 0;
        float ax, ay, az, gx, gy, gz;

        // Throw away the first 50 samples to let the physical silicon settle
        for (int i = 0; i < 50; i++) {
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
            delay(5); 
        }

        offsetAx = sumAx / samples;
        offsetAy = sumAy / samples;
        offsetAz = (sumAz / samples) - 1000.0f; 
        
        offsetGx = sumGx / samples;
        offsetGy = sumGy / samples;
        offsetGz = sumGz / samples;
    }

    bool read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        ISDS_state_t accReady, gyroReady;
        ISDS_isDataReady(&sensorInterface, NULL, &accReady, &gyroReady);
        
        if (accReady == ISDS_enable && gyroReady == ISDS_enable) {
            readRaw(ax, ay, az, gx, gy, gz);
            
            // Subtract offsets
            ax -= offsetAx;
            ay -= offsetAy;
            az -= offsetAz;
            gx -= offsetGx;
            gy -= offsetGy;
            gz -= offsetGz;
            
            // Convert to matching physics units (NO FILTERING FOR EKF!)
            ax = (ax / 1000.0f) * 9.81f;
            ay = (ay / 1000.0f) * 9.81f;
            az = (az / 1000.0f) * 9.81f;

            gx = (gx / 1000.0f) * (PI / 180.0f);
            gy = (gy / 1000.0f) * (PI / 180.0f);
            gz = (gz / 1000.0f) * (PI / 180.0f);

            return true;
        }
        return false;
    }

private:
    void readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        ISDS_getAccelerations_float(&sensorInterface, &ax, &ay, &az);
        ISDS_getAngularRates_float(&sensorInterface, &gx, &gy, &gz);
    }
};

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