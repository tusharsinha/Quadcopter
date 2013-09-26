#include <Wire.h>

/* -------------------------------------- HMC5883L addresses -------------------------------------- */

#define HMC5883L_ADDR    		0x1E
#define ConfigurationRegisterA  0x00
#define ConfigurationRegisterB  0x01
#define ModeRegister 			0x02
#define DataRegisterBegin 		0x03

int _buff[8];
int xmagn = 0, ymagn = 0, zmagn = 0;
int GainMagn = 1090;
int GainMagn = 1090;


void setup(){
  Serial.begin(9600);
  Wire.begin();
  initialiseHMC5883L();
}

void loop(){
  readMagn(&xmagn, &ymagn, &zmagn); //read the magnetometer values and store them in variables  x,y,z
  Serial.print(xmagn);
  Serial.print(ymagn);
  Serial.println(zmagn);  
}

void initialiseHMC5883L() {
   writeTo(HMC5883L_ADDR, ConfigurationRegisterA, B01110000);
   /*
   D7 - 1/0 have to check
   D6 D5 - number of readings to be averaged 00 - 1; 01 - 2; 10 - 4; 11 - 8
   D4 D3 D2 - data output rate 100 - 15Hz 101 -30Hz 110 75hz
   D1 D0 - 00 default 01 self test +ve 10 self test -ve 11 reserved
   */
   writeTo(HMC5883L_ADDR, ConfigurationRegisterB, B00100000); 
   /* 
   D7 D6 D5
   001 - 1.3 Ga gain 1090 default
	gauss = 0.88regValue = 0x00	m_Scale = 0.73
	gauss = 1.3 regValue = 0x01 m_Scale = 0.92
	gauss = 1.9 regValue = 0x02 m_Scale = 1.22
	gauss = 2.5 regValue = 0x03 m_Scale = 1.52
	gauss = 4.0 regValue = 0x04	m_Scale = 2.27
	gauss = 4.7 regValue = 0x05	m_Scale = 2.56
	gauss = 5.6 regValue = 0x06	m_Scale = 3.03
	gauss = 8.1 regValue = 0x07 m_Scale = 4.35
   */
   writeTo(HMC5883L_ADDR, ModeRegister, B0000000);
   /*
   D7 - D2 = 0
   D1 D0 = 00 - continuous ; 01 - Single measurement mode;
   */
}

// Reads the acceleration into three variable x, y and z
void readMagn(int *x, int *y, int *z) {
  readFrom(HMC5883L_ADDR, DataRegisterBegin, 6, _buff); //read the magnetic field data from the HMC5883L
  // each axis reading comes in 12 bit resolution in 2 bytes.  MOST Significant Byte first!!
  
  *x = (((int)_buff[0] << 8) | _buff[1])/GainMagn;
  *y = (((int)_buff[5] << 8) | _buff[6])/GainMagn; 
  *z = (((int)_buff[3] << 8) | _buff[4])/GainMagn;
  
  heading = atan2(*y, *x);
  heading += 0.009308;  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
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