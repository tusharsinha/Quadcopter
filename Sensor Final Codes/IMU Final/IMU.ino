#include <Wire.h>

/* -------------------------------------- ADXL345 addresses -------------------------------------- */

#define ADXL345_ADDR 0x1D 
// 0x1D - ADXL345 address when ALT is connected to HIGH
// 0x53 - ADXL345 address when ALT is connected to LOW

/* ------- Register names ------- */
#define ADXL345_DEVID 0x00 			 // Device ID
#define ADXL345_RESERVED1 0x01		 // 0x01 to 0x1C reserved : DO NOT ACCESS!
#define ADXL345_OFSX 0x1e			 // X , Y and Z offset used to correct readings
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_BW_RATE 0x2c		 // Data rate and power mode control D4 = lowpower; D3,2,1,0 - for bit rates defined below)
#define ADXL345_POWER_CTL 0x2d		 // Power saving features control
#define ADXL345_DATA_FORMAT 0x31	 
#define ADXL345_DATAX0 0x32 		// 0x32 - 0x37 is output depending on Data format bit
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37

#define ADXL345_FIFO_CTL 0x38		// FIFO control
#define ADXL345_FIFO_STATUS 0x39	// FIFO status
/*will change default FIFO Modes if needed presently no change*/
/*Default data format 000000xx*/

/* ------ other definitions Pins,bits ------ */
#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111
#define ADXL345_BW_3    0x6 // 0110

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

/* ------------------------------ End of Address and bits Deceleration ------------------------------*/

int xgyro = 0, ygyro = 0, zgyro = 0; //gyroscope raw output
int xaccel = 0, yaccel = 0, zaccel = 0; //accelerometer raw output
int xoffgyro = 0, yoffgyro = 0, zoffgyro = 0; //stores gyro offset as there is not dedicated register for it
int xoffset = 0, yoffset = 0, zoffset = 0; //for offset tuning average calculation
int _buff[8]; //used in burst reading

int error_code = 0; //Define error flags here
#define ADXL345_READ_ERROR 1 //if less number of bytes is read from accelerometer
#define ITG3200_READ_ERROR 2 //if less number of bytes is read from gyroscope

bool fastmode false; //enable/disable disable fast mode
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
  //might not be the case with DUE
  initialiseADXL345();
  initialiseITG3200();
}

void loop(){
  readGyro(&xgyro, &ygyro, &zgyro); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(xgyro);
  Serial.print(xgyro);
  Serial.println(xgyro);
  readAccel(&xaccel, &yaccel, &zaccel); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(xaccel);
  Serial.print(yaccel);
  Serial.println(zaccel);
  delay(1000);
}

// Initialise Accelerometer
void initialiseADXL345() {
  writeTo(ADXL345_ADDR, ADXL345_POWER_CTL, 8);
  /*
  D7, D6  = 0
  D5 = Link default 0
  D4 = Auto Sleep
  D3 = Meausre
  D2 = sleep
  D1, D0 = Wakeup
  8 means only measure mode
  */
  writeTo(ADXL345_ADDR, ADXL345_BW_RATE, ADXL345_BW_100); //select bandwidth and transfer rate here (1600,800,400,200,100,50,25,12,6,3)
  // note - if you change resolution change z offset correction accordingly
  writeTo(ADXL345_ADDR, ADXL345_DATA_FORMAT, B00000001); //select range D1,D0 = range ( 00 = 2g ; 01 = 4g; 10 = 8g; 11 = 16g)
  /* 
  D7 = self test 0 - disable
  D6 = SPI bit 1 sets to 3 wire and 0 to 4 wire
  D5 = interrupt invert 0 is active HIGH
  D4 = 0
  D3 = Full resolution bit 10 bit if 0 other wise max bits corresponding to resolution selected
  D4 = Justify bit set the order of bit stream right justified = 0
  D1,D0 = range ( 00 = 2g ; 01 = 4g; 10 = 8g; 11 = 16g)
  */
  int i = 1;
  int n = 10; //Change n to increase number of samples to average to remove offset
  while(i<=n){
	readAccel(&xaccel, &yaccel, &zaccel);
	xoffset += xaccel;
	yoffset += yaccel;
	zoffset += zaccel;
	i++;
	delay(10);
  }
  xoffset = (xoffset/n);
  yoffset = (yoffset/n);
  zoffset = (zoffset/n);
  //zoffset correction
  zoffset = zoffset - 256; //for 2g resolution at 10bit
  writeTo(ADXL345_ADDR, ADXL345_OFSX, xoffset);
  writeTo(ADXL345_ADDR, ADXL345_OFSY, yoffset);
  writeTo(ADXL345_ADDR, ADXL345_OFSZ, zoffset);
}

// Reads the acceleration into three variable x, y and z
void readAccel(int *x, int *y, int *z) {
  readFrom(ADXL345_ADDR, ADXL345_DATAX0, 6, _buff); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution in 2 bytes.  Least Significat Byte first!!
  
  *x = (((int)_buff[1]) << 8) | _buff[0];  
  *y = (((int)_buff[3]) << 8) | _buff[2];
  *z = (((int)_buff[5]) << 8) | _buff[4];
}

// Initialises Gyroscope
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
  writeTo(ITG3200_ADDR, PWR_MGM, B00000001 ); 
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
  //making data available ready as interrupt
  /*
  D7 = Active low
  D6 = open drain(1)/push-pull(0)
  D5 = Latch mode
  D4 = Latch clear method
  D2 = Enable interrupt when device is ready
  D0 = Enable interrupt when data is available
  */
  delay(GYROSTART_UP_DELAY);  // start up delay
  int i = 1;
  int n = 10; //Change n to increase number of samples to average to remove offset
  xoffset = 0;
  yoffset = 0;
  zoffset = 0;
  while(i<=n){
    readGyro(&xgyro, &ygyro, &zgyro);
    xoffset += xgyro;
    yoffset += ygyro;
    zoffset += zgyro; 
	i++;
	delay(10);	
  }
  xoffgyro = xoffset/n;
  yoffgyro = yoffset/n;
  zoffgyro = zoffset/n;
}

// Reads the angular acceleration into three variable x, y and z
void readGyro(int *x, int *y, int *z) {
  int x, y, z;
  // x,y,z will contain calibrated integer values from the sensor
  readFrom(ITG3200_ADDR, GYRO_XOUT, 6, _buff);  //read the gyroscope data from ITG3200
  // each axis reading comes in 16 bit resolution in 2 bytes.  Most Significat Byte first!!
  
  *x = (((int)_buff[0] << 8) | _buff[1]);
  *y = (((int)_buff[2] << 8) | _buff[3]); 
  *z = (((int)_buff[4] << 8) | _buff[5]);
  
  *x =  (x / 14.375) - xoffgyro;
  *y =  (y / 14.375) - yoffgyro;
  *z =  (z / 14.375) - zoffgyro;
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
    if(dev_address == ADXL345_ADDR)
	{
		error_code = ADXL345_READ_ERROR;
	}
	else
	{
		error_code = ITG3200_READ_ERROR;
	}
  }
  Wire.endTransmission();         // end transmission
}
