
#include IMU.h

ADXL345* IMU::getAccelerometer() {
    return &accelerometer;
}

HMC5883L* IMU::getMagnetometer() {
    return &magnetometer;
}

ITG3200* IMU::getGyroscope() {
    return &gyroscope;
}