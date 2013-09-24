#include <Wire.h>

/* -------------------------------------- ITG3200 addresses -------------------------------------- */

#define ITG3200_ADDR  0x69
//	AD0=1 0x69 I2C address when AD0 is connected to HIGH (VCC) - default for sparkfun breakout
//	AD0=0 0x68 I2C address when AD0 is connected to LOW (GND)
//  The LSB bit of the 7 bit address is determined by the logic level on pin 9. 
#define GYROSTART_UP_DELAY  100    // 50ms from gyro startup + 20ms register r/w startup + 30ms to be on safe side

/* -------- Register names -------- */
#define WHO_AM_I           0x00  // RW  SETUP: I2C address   
#define SMPLRT_DIV         0x15  // RW  SETUP: Sample Rate Divider       
#define DLPF_FS            0x16  // RW  SETUP: Digital Low Pass Filter/ Full Scale range
#define INT_CFG            0x17  // RW  Interrupt: Configuration
#define INT_STATUS         0x1A  // R	Interrupt: Status
#define TEMP_OUT           0x1B  // R	SENSOR: Temperature 2bytes
#define GYRO_XOUT          0x1D  // R	SENSOR: Gyro X 2bytes  
#define GYRO_YOUT          0x1F  // R	SENSOR: Gyro Y 2bytes
#define GYRO_ZOUT          0x21  // R	SENSOR: Gyro Z 2bytes
#define PWR_MGM            0x3E  // RW	Power Management

int xgyro = 0, ygyro = 0, zgyro = 0;
float gains[3]; 
int offsets[3];
float polarities[3];
int _buff[8];
bool fastmode false; //enable disable fast mode
//boolean fastmode true;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  if(fastmode) { // switch to 400KHz I2C
    TWBR = ((16000000L / 400000L) - 16) / 2; 
	/*
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2
	SCL frequency = CPU_FREQUENCY / (16 + 2(TWBR) * (PrescalerValue))
	16000000 / (16 + 2 * 72) = 100000
	But if we change TWBR to 12 we get:
	16000000 / (16 + 2 * 12) = 400000
	*/
  }
  //need to check few things
  //wire automatically pulls up on start
  //might not be the case with due
  
  initialiseITG3200();
}

void loop(){
  readGyro(&xgyro, &ygyro, &zgyro,); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(xgyro);
  Serial.print(xgyro);
  Serial.println(xgyro);  
}

void initialiseITG3200() {
  writeTo(ITG3200_ADDR, SMPLRT_DIV, 0);
  // default = 0 i.e no scale driver    FsampleHz=SampleRateHz/(divider+1) range 0-255
  readFrom(ITG3200_ADDR, DLPF_FS, 1, &_buff[0]);   
  writeTo(ITG3200_ADDR, DLPF_FS, ((_buff[0] & B11100000) | B00011000) );
  /*
  D7 D6 D5 reserved
  D4 D3 full scale selection must be set 11 for proper operation
  D2 D1 D0 low pass filter configuration and sampling rate
  BW256 SR8           0   // default    256Khz BW and 8Khz SR
  BW188 SR1           1
  BW098 SR1           2
  BW042 SR1           3
  BW020 SR1           4
  BW010 SR1           5
  BW005 SR1           6
  */
  readFrom(ITG3200_ADDR, PWR_MGM, 1, &_buff[0]);
  writeTo(ITG3200_ADDR, PWR_MGM, ((_buff[0] & B10000000) | B00000001) ); 
  /*
  D7 = H_reset
  D6 = Sleep default = 0 (no power saving)
  D5, D4, D3 = standby X, Y, Z (default 000 )
  D2 D1 D0 = clock selection
  Internal oscillator  0   // default
  PLL XGYRO REF       1
  PLL YGYRO REF       2
  PLL ZGYRO REF       3
  PLL EXTERNAL32      4   // 32.768 kHz
  PLL EXTERNAL19      5   // 19.2 Mhz
  Reserved 			  6,7
  */
  readFrom(ITG3200_ADDR, INT_CFG, 1, &_buff[0]);
  writeTo(ITG3200_ADDR, INT_CFG, ((_buff[0] & B11111010) | B00000101) ); 
  //making data available ready as interrupt though won't need it
  /*
  D7 = Active low
  D6 = open drain(1)/push-pull(0)
  D5 = Latch mode
  D4 = Latch clear method
  D2 = Enable interrupt when device is ready
  D0 = Enable interrupt when data is available
  */
  delay(GYROSTART_UP_DELAY);  // startup delay
  setGains(1.0,1.0,1.0);
  setRevPolarity(0,0,0);
  zeroCalibrate(10,10);
}

void setGains(float _Xgain, float _Ygain, float _Zgain) {
  gains[0] = _Xgain;
  gains[1] = _Ygain;
  gains[2] = _Zgain;
}

void setOffsets(int _Xoffset, int _Yoffset, int _Zoffset) {
  offsets[0] = _Xoffset;
  offsets[1] = _Yoffset;
  offsets[2] = _Zoffset;
}

void setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
  polarities[0] = _Xpol ? -1 : 1;
  polarities[1] = _Ypol ? -1 : 1;
  polarities[2] = _Zpol ? -1 : 1;
}

void zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  int xyz[3]; 
  float tmpOffsets[] = {0,0,0};

  for (int i = 0;i < totSamples;i++){
    delay(sampleDelayMS);
    readGyro(xyz);
    tmpOffsets[0] += xyz[0]/ totSamples;
    tmpOffsets[1] += xyz[1]/ totSamples;
    tmpOffsets[2] += xyz[2]/ totSamples;  
  }
  setOffsets(tmpOffsets[0], tmpOffsets[1], tmpOffsets[2]);
}

// Reads the angular acceleration into three variable x, y and z
void readGyro(int *x, int *y, int *z) {
  int x, y, z;
  // x,y,z will contain calibrated integer values from the sensor
  readFrom(ITG3200_ADDR, GYRO_XOUT, 6, _buff);
  *_GyroX = ((_buff[0] << 8) | _buff[1]);
  *_GyroY = ((_buff[2] << 8) | _buff[3]); 
  *_GyroZ = ((_buff[4] << 8) | _buff[5]);
  
  *_GyroX =  x / 14.375 * polarities[0] * gains[0];
  *_GyroY =  y / 14.375 * polarities[1] * gains[1];
  *_GyroZ =  z / 14.375 * polarities[2] * gains[2];
  *_GyroX -= offsets[0];
  *_GyroY -= offsets[1];
  *_GyroZ -= offsets[2];
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