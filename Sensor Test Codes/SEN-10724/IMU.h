
#ifndef IMU_H
#define IMU_H

//The magnetometer
#include HMC5883L.h
//The accelerometer
#include ADXL345.h
//The gyroscope
#include ITG3200.h
class IMU {
public:
    ADXL345*        getAccelerometer();
    HMC5883L*       getMagnetometer();
    ITG3200*        getGyroscope();
private:
    ADXL345         accelerometer;
    HMC5883L        magnetometer;
    ITG3200         gyroscope;
};

#endif