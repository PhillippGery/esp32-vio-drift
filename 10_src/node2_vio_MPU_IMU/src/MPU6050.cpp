#include "MPU6050.h"

bool MPU6050::begin(uint8_t addr) {
  _addr = addr;
  _writeReg(0x6B, 0x00);  // wake up
  delay(100);
  _writeReg(0x1C, 0x00);  // accel ±2g
  _writeReg(0x1B, 0x00);  // gyro ±250°/s
  return true;
}

bool MPU6050::readRaw(float &ax, float &ay, float &az,
                      float &gx, float &gy, float &gz) {
  uint8_t buf[14];
  
  Wire.beginTransmission(_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false; // I2C communication failed
  }
  
  if (Wire.requestFrom(_addr, (uint8_t)14) != 14) {
    return false; // Did not receive expected bytes
  }
  
  for (int i = 0; i < 14; i++) {
    buf[i] = Wire.read();
  }

  ax = _toInt(buf[0],  buf[1])  / ACCEL_SCALE * 9.81f;
  ay = _toInt(buf[2],  buf[3])  / ACCEL_SCALE * 9.81f;
  az = _toInt(buf[4],  buf[5])  / ACCEL_SCALE * 9.81f;
  gx = _toInt(buf[8],  buf[9])  / GYRO_SCALE;
  gy = _toInt(buf[10], buf[11]) / GYRO_SCALE;
  gz = _toInt(buf[12], buf[13]) / GYRO_SCALE;

  return true;
}

bool MPU6050::read(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz) {
  float rax, ray, raz, rgx, rgy, rgz;
  
  if (!readRaw(rax, ray, raz, rgx, rgy, rgz)) {
    return false;
  }
  
  ax = _fax.update(rax - offsetAx);
  ay = _fay.update(ray - offsetAy);
  az = _faz.update(raz - offsetAz);
  gx = _fgx.update(rgx - offsetGx);
  gy = _fgy.update(rgy - offsetGy);
  gz = _fgz.update(rgz - offsetGz);

  return true;
}

void MPU6050::calibrate(int samples, int delayMs) {
  double sx = 0, sy = 0, sz = 0, sgx = 0, sgy = 0, sgz = 0;
  float ax, ay, az, gx, gy, gz;
  
  delay(500); // Allow sensor to settle
  
  for (int i = 0; i < samples; i++) {
    readRaw(ax, ay, az, gx, gy, gz);
    sx += ax; 
    sy += ay; 
    sz += az;
    sgx += gx; 
    sgy += gy; 
    sgz += gz;
    delay(delayMs);
  }
  
  offsetAx = sx / samples;
  offsetAy = sy / samples;
  offsetAz = (sz / samples) - 9.81f; // Account for gravity
  offsetGx = sgx / samples;
  offsetGy = sgy / samples;
  offsetGz = sgz / samples;
}

void MPU6050::_writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t MPU6050::_toInt(uint8_t h, uint8_t l) {
  return (int16_t)((h << 8) | l);
}