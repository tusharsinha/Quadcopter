#include <Wire.h>

/* -------------------------------------- ADXL345 addresses -------------------------------------- */

#define HMC5883L_ADDR    		0x1E
#define ConfigurationRegisterA  0x00
#define ConfigurationRegisterB  0x01
#define ModeRegister 			0x02
#define DataRegisterBegin 		0x03

int _buff[8];
int xmagn = 0, ymagn = 0, zmagn = 0;
int GainMagn = 1.0;


void setup(){
  Serial.begin(9600);
  Wire.begin();
  initialiseHMC5883L();
}

void loop(){
  readMagn(&xmagn, &ymagn, &zmagn); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(xmagn);
  Serial.print(ymagn);
  Serial.println(zmagn);  
}

void initialiseHMC5883L(){
	
}

// Reads the acceleration into three variable x, y and z
void readMagn(int *x, int *y, int *z) {
  readFrom(HMC5883L_ADDR, DataRegisterBegin, 6, _buff); //read the magnetic field data from the HMC5883L
  // each axis reading comes in 12 bit resolution in 2 bytes.  MOST Significant Byte first!!
  
  *x = (((int)_buff[0] << 8) | _buff[1])/GainMagn;
  *y = (((int)_buff[5] << 8) | _buff[6])/GainMagn; 
  *z = (((int)_buff[3] << 8) | _buff[4])/GainMagn;
  }


// Writes val to address register on device
void writeTo(int dev_address, byte address, byte val) {
  Wire.beginTransmission(dev_address); // start transmission to device
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array (BURST READING)
void readFrom(int dev_address, byte address, int num, byte _buff[]) {
  Wire.beginTransmission(dev_address); // start transmission to device
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(dev_address); // start transmission to device
  Wire.requestFrom(dev_address, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  {
    _buff[i] = Wire.read();    // receive a byte
    i++;
  }
  if(i != num){
    error_code = ADXL345_READ_ERROR;
  }
  Wire.endTransmission();         // end transmission
}