#include <Wire.h>

/* -------------------------------------- ADXL345 addresses -------------------------------------- */

#define ADXL345_ADDR 0x53 
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

/* -------------------------------------- ITG3200 addresses -------------------------------------- */

#define ITG3200_ADDR  0x68
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

/* -------------------------------------- HMC5883L addresses -------------------------------------- */

#define HMC5883L_ADDR    		0x1E
#define ConfigurationRegisterA  0x00
#define ConfigurationRegisterB  0x01
#define ModeRegister 			0x02
#define DataRegisterBegin 		0x03

/* ---------------------------------- End of Address Deceleration -----------------------------------*/

// 1 - x axis ; 2 - y axis ; 3 - z axis

int gyro_raw[] = {0,0,0}; //gyroscope output
float gyro_final[] = {0,0,0};
float gyro_offset[] = {0,0,0}; //stores gyroscope offset as there is not dedicated register for it

int accel_raw[] = {0,0,0}; //accelerometer output
float accel_final[] = {0,0,0}; //in g

int magn_raw[] = {0,0,0}; //magnetometer output
float magn_final[] = {0,0,0};
float heading = 0;
int GainMagn = 1090; //Gain of magnetometer

float offset[] = {0,0,0}; //for offset tuning average calculation

byte _buff[6]; //used in burst reading

int error_code = 0; //Define error flags here
#define ADXL345_READ_ERROR 1 //if less number of bytes is read from accelerometer
#define ITG3200_READ_ERROR 2 //if less number of bytes is read from gyroscope
#define HMC5883L_READ_ERROR 3 //if less number of bytes is read from magnetometer

//boolean fastmode = false; //enable/disable disable fast mode
//boolean fastmode true;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  /*if(fastmode) { // switch to 400KHz I2C
    TWBR = ((16000000L / 400000L) - 16) / 2; 
	/*
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2
	SCL frequency = CPU_FREQUENCY / (16 + 2(TWBR) * (PrescalerValue))
	16000000 / (16 + 2 * 72) = 100000
	But if we change TWBR to 12 we get:
	16000000 / (16 + 2 * 12) = 400000
	
  }
  need to check few things*/  
  initialiseADXL345();
  initialiseITG3200();
  initialiseHMC5883L();
}

void loop(){
  readGyro(); //read the gyroscope values and store them in variables  x,y,z
  Serial.print(gyro_final[0]);
  Serial.print(gyro_final[1]);
  Serial.println(gyro_final[2]);
  readAccel(); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(accel_final[0]);
  Serial.print(accel_final[1]);
  Serial.println(accel_final[2]);
  readMagn(); //read the magnetometer values and store them in variables  x,y,z
  Serial.print(magn_final[0]);
  Serial.print(magn_final[1]);
  Serial.println(magn_final[2]);  
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
  writeTo(ADXL345_ADDR, ADXL345_BW_RATE, B1011);
  /*
  ADXL345_BW_1600 0xF // 1111
  ADXL345_BW_800  0xE // 1110
  ADXL345_BW_400  0xD // 1101  
  ADXL345_BW_200  0xC // 1100
  ADXL345_BW_100  0xB // 1011  
  ADXL345_BW_50   0xA // 1010 
  ADXL345_BW_25   0x9 // 1001 
  ADXL345_BW_12   0x8 // 1000 
  ADXL345_BW_6    0x7 // 0111
  ADXL345_BW_3    0x6 // 0110
  */
  
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
	readAccel();
	offset[1] += accel_raw[1];
	offset[2] += accel_raw[2];
	offset[3] += accel_raw[3];
	i++;
	delay(10);
  }
  offset[1] = (offset[1]/n);
  offset[2] = (offset[2]/n);
  offset[3] = (offset[3]/n);
  //zoffset correction
  offset[3] = offset[3] - 256; //for 2g resolution at 10bit
  writeTo(ADXL345_ADDR, ADXL345_OFSX, offset[1]);
  writeTo(ADXL345_ADDR, ADXL345_OFSY, offset[2]);
  writeTo(ADXL345_ADDR, ADXL345_OFSZ, offset[3]);
}

// Reads the acceleration into three variable x, y and z
void readAccel() {
  readFrom(ADXL345_ADDR, ADXL345_DATAX0, 6, _buff); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution in 2 bytes.  Least Significat Byte first!!
  
  accel_raw[1] = (((int)_buff[1] << 8) | _buff[0]);  
  accel_raw[2] = (((int)_buff[3] << 8) | _buff[2]);
  accel_raw[3] = (((int)_buff[5] << 8) | _buff[4]);
  accel_final[1] = accel_raw[1]/256;
  accel_final[2] = accel_raw[2]/256;
  accel_final[3] = accel_raw[3]/256;
}

// Initialises Magnetometer
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

// Reads the magnetic field into three variable x, y and z
void readMagn() {
  readFrom(HMC5883L_ADDR, DataRegisterBegin, 6, _buff); //read the magnetic field data from the HMC5883L
  // each axis reading comes in 12 bit resolution in 2 bytes.  MOST Significant Byte first!!
  
  magn_raw[1] = (((int)_buff[0] << 8) | _buff[1]);
  magn_raw[2] = (((int)_buff[5] << 8) | _buff[6]); 
  magn_raw[3] = (((int)_buff[3] << 8) | _buff[4]);
  
  magn_final[1] = magn_raw[1]/GainMagn;
  magn_final[2] = magn_raw[2]/GainMagn;
  magn_final[3] = magn_raw[3]/GainMagn;
  
  heading = atan2(magn_raw[2], magn_raw[1]);
  heading += 0.009308;  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
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
  while(i<=n){
    readGyro();
    offset[1] += gyro_raw[1];
    offset[2] += gyro_raw[2];
    offset[3] += gyro_raw[3]; 
	i++;
	delay(10);	
  }
  gyro_offset[1] = offset[1]/n;
  gyro_offset[2] = offset[2]/n;
  gyro_offset[3] = offset[3]/n;
}

// Reads the angular acceleration into three variable x, y and z
void readGyro() {
  
  readFrom(ITG3200_ADDR, GYRO_XOUT, 6, _buff);  //read the gyroscope data from ITG3200
  // each axis reading comes in 16 bit resolution in 2 bytes.  Most Significat Byte first!!
  
  gyro_raw[1] = (((int)_buff[0] << 8) | _buff[1]);
  gyro_raw[2] = (((int)_buff[2] << 8) | _buff[3]); 
  gyro_raw[3] = (((int)_buff[4] << 8) | _buff[5]);
  
  gyro_final[1] =  ((gyro_raw[1] - gyro_offset[1])/ 14.375);
  gyro_final[2] =  ((gyro_raw[2] - gyro_offset[2])/ 14.375);
  gyro_final[3] =  ((gyro_raw[3] - gyro_offset[3])/ 14.375);
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
