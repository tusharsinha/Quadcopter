#include <Wire.h>

#define HMC5883L_Address 		0x1E
#define ConfigurationRegisterA  0x00
#define ConfigurationRegisterB  0x01
#define ModeRegister 			0x02
#define DataRegisterBegin 		0x03

#define Measurement_Continuous  0x00
#define Measurement_SingleShot  0x01
#define Measurement_Idle 		0x03

