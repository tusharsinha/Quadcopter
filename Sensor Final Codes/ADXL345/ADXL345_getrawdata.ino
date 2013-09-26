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

int xaccel = 0,yaccel = 0,zaccel = 0;
int xoffset = 0, yoffset = 0, zoffset = 0;
int _buff[8];

int error_code = 0;
#define ADXL345_READ_ERROR 1 //if less number of bytes

void setup(){
  Serial.begin(9600);
  Wire.begin();
  initialiseADXL345();
}

void loop(){
  readAccel(&xaccel, &yaccel, &zaccel); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(xaccel);
  Serial.print(yaccel);
  Serial.println(zaccel);  
}

void initialiseADXL345(){
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
  writeTo(ADXL345_ADDR, ADXL345_BW_RATE, ADXL345_BW_1600); //select resolution here (1600,800,400,200,100,50,25,12,6,3)
  // note - if you change resolution change z offset correction accordingly
  writeTo(ADXL345_ADDR, ADXL345_DATA_FORMAT, B00000001); //select range D1,D0 = range ( 00 = 2g ; 01 = 4g; 10 = 8g; 11 = 16g)
  /* not separating bits because we need to write whole registers
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
	xoffset += (xaccel/n);
	yoffset += (yaccel/n);
	zoffset += (zaccel/n);
  }
  //zoffset correction
  zoffset = zoffset - 512; //for 2g resolution at 10bit
  writeTo(ADXL345_ADDR, ADXL345_OFSX, xoffset);
  writeTo(ADXL345_ADDR, ADXL345_OFSY, yoffset);
  writeTo(ADXL345_ADDR, ADXL345_OFSZ, zoffset);
}

// Reads the acceleration into three variable x, y and z
void readAccel(int *x, int *y, int *z) {
  readFrom(ADXL345_ADDR, ADXL345_DATAX0, 6, _buff); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  *x = (((int)_buff[1]) << 8) | _buff[0];  
  *y = (((int)_buff[3]) << 8) | _buff[2];
  *z = (((int)_buff[5]) << 8) | _buff[4];
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