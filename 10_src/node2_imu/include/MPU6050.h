#pragma once
#include <Wire.h>
#include <Arduino.h>

/**
 * @brief Template class for calculating a moving average.
 * @tparam N Number of samples in the moving average window.
 */
template<int N = 5>
class MovingAvg {
  float buf[N] = {};
  int idx = 0;
  bool full = false;

public:
  /**
   * @brief Update the moving average with a new value.
   * * @param val The new reading to add to the buffer.
   * @return The current moving average.
   */
  float update(float val) {
    buf[idx] = val;
    idx = (idx + 1) % N;
    if (idx == 0) full = true;
    
    int count = full ? N : idx;
    float sum = 0;
    for (int i = 0; i < count; i++) {
      sum += buf[i];
    }
    return sum / count;
  }
};

/**
 * @brief Driver class for the MPU-6050 Accelerometer and Gyroscope.
 */
class MPU6050 {
public:
  float offsetAx = 0, offsetAy = 0, offsetAz = 0;
  float offsetGx = 0, offsetGy = 0, offsetGz = 0;

  static constexpr float ACCEL_SCALE = 13148.0f; 
  static constexpr float GYRO_SCALE  = 131.0f;

  /** * @brief Initialize the MPU-6050 sensor.
   * * @param addr I2C address of the device (default: 0x68)
   * @return true on successful initialization, false otherwise
   */
  bool begin(uint8_t addr = 0x68);

  /** * @brief Read raw (unfiltered and non-offset) data from MPU-6050.
   * * @param[out] ax  Acceleration X in m/s²
   * @param[out] ay  Acceleration Y in m/s²
   * @param[out] az  Acceleration Z in m/s²
   * @param[out] gx  Angular rate X in deg/s
   * @param[out] gy  Angular rate Y in deg/s
   * @param[out] gz  Angular rate Z in deg/s
   * @return true on success, false if I²C read failed 
   */
  bool readRaw(float &ax, float &ay, float &az,
               float &gx, float &gy, float &gz);
  bool readRaw(float &ax, float &ay, float &az,
               float &gx, float &gy, float &gz,
               float &temp_c);

  /** * @brief Read filtered and offset-compensated data from MPU-6050.
   * * @param[out] ax  Acceleration X in m/s²
   * @param[out] ay  Acceleration Y in m/s²
   * @param[out] az  Acceleration Z in m/s²
   * @param[out] gx  Angular rate X in deg/s
   * @param[out] gy  Angular rate Y in deg/s
   * @param[out] gz  Angular rate Z in deg/s
   * @return true on success, false if I²C read failed 
   */
  bool read(float &ax, float &ay, float &az,
            float &gx, float &gy, float &gz);
  bool read(float &ax, float &ay, float &az,
            float &gx, float &gy, float &gz,
            float &temp_c);

  /** * @brief Calculate zero-motion offsets for calibration.
   * * @param samples  Number of samples to average (default: 500)
   * @param delayMs  Delay between samples in milliseconds (default: 10)
   */
  void calibrate(int samples = 500, int delayMs = 10);

private:
  uint8_t _addr;
  MovingAvg<5> _fax, _fay, _faz, _fgx, _fgy, _fgz;

  void _writeReg(uint8_t reg, uint8_t val);
  static int16_t _toInt(uint8_t h, uint8_t l);
};