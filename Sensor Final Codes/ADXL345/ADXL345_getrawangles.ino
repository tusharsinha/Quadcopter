#include <Wire.h>
#include <ADXL345.h>

int ADXL345_Address = 29; // ADXL345 address when ALT is connected to HIGH
int ADXL345_Address = 83; // ADXL345 address when ALT is connected to LOW
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library see class ADXL345 in ADXL345.h 
int x = 0,y = 0,z = 0;
int xoffset = 0, yoffset = 0, zoffset = 0;
int offsetnsample = 10;
float ADXL345_Rate = 1400; // 3<X<1400
int ADXL345_Res = 2; // 2,4,8,16 default 2

void setup(){
  Serial.begin(9600);
  Wire.begin();
  initialiseADXL345();
}

void loop(){
  adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
  Serial.print(x);
  Serial.print(y);
  Serial.println(z);  
}

/*will change default FIFO Modes if needed presently no change*/
/*Default data format 000000xx*/

void initialiseADXL345(resolution,bitrate){
  adxl.init(ADXL345_Address);
  adx1.setRate(ADXL345_Rate);
  adx1.setRangeSetting(ADXL345_Res);
  int i = 1;
  while(i<=n){
	adxl.readAccel(&x, &y, &z);
	xoffset = xoffset + (x/n);
	yoffset = yoffset + (y/n);
	zoffset = zoffset + (z/n);
  }
  //zoffset correction
  zoffset = zoffset - 512; //for 2g resolution at 10bit
  adx1.setAxisOffset(xoffset, yoffset, zoffset);
}