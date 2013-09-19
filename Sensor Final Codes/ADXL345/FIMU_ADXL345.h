/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/
#include "Arduino.h"

#ifndef FIMU_ADXL345_h
#define FIMU_ADXL345_h

/* -- ADXL345 addresses --*/
#define ADXL345_ADDR_ALT_HIGH 0x1D // ADXL345 address when ALT is connected to HIGH
#define ADXL345_ADDR_ALT_LOW  0x53 // ADXL345 address when ALT is connected to LOW

/* ------- Register names ------- */
#define ADXL345_DEVID 0x00 			 // Device ID
#define ADXL345_RESERVED1 0x01		 // 0x01 to 0x1C reserved : do not access 
#define ADXL345_THRESH_TAP 0x1d		 // Tap Threshold limit
#define ADXL345_OFSX 0x1e			 // X , Y and Z offset used to correct readings
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21			 // Minimum duration of Tap
#define ADXL345_LATENT 0x22			 // Time between Tap and start of Window 	
#define ADXL345_WINDOW 0x23			 // Duration of Window
#define ADXL345_THRESH_ACT 0x24		 // Activity Threshold
#define ADXL345_THRESH_INACT 0x25	 // Inactivity Threshold
#define ADXL345_TIME_INACT 0x26		 // Inactivity Time
#define ADXL345_ACT_INACT_CTL 0x27   // Activity Inactivity control (D7 - Act ac/dc; D 6,5,4 active x,y,z; next 4 corresponding Inact)
#define ADXL345_THRESH_FF 0x28		 // Free fall threshold
#define ADXL345_TIME_FF 0x29		 // Free fall Time (Minimum)
#define ADXL345_TAP_AXES 0x2a		 // Enable tap on axes ( D7 - D4 = 0 ; D3 = for supressing double taps; D2, D1, D0 x,y,z)
#define ADXL345_ACT_TAP_STATUS 0x2b  // Source of activity or Tap ( D7 =0; D 6,5,4 - ACT x,y,z; D3= Asleep; D 2,1,0 - TAP x,y,z)
#define ADXL345_BW_RATE 0x2c		 // Data rate and power mode control D4 = lowpower; D3,2,1,0 - for bit rates defined below)
#define ADXL345_POWER_CTL 0x2d		 // Power saving features control
#define ADXL345_INT_ENABLE 0x2e		 // Interrupt Enable control
#define ADXL345_INT_MAP 0x2f		 // Interrupt Mapping control ored	
#define ADXL345_INT_SOURCE 0x30      // Source of Interrupt
#define ADXL345_DATA_FORMAT 0x31	 
/* not separating bits because we need to write whole registers
D7 = selftest 0 - disable
D6 = SPI bit 1 sets to 3 wire and 0 to 4 wire
D5 = interrupt invert 0 is active HIGH
D4 = 0
D3 = Full resolution bit 10 bit if 0 other wise max bits corresponding to resolution selected
D4 = Justify bit set the order of bit stream right justified = 0
D1,D0 = range ( 00 = 2g ; 01 = 4g; 10 = 8g; 11 = 16g)
*/

#define ADXL345_DATAX0 0x32 		// 0x32 - 0x37 is output depending on Data format bit
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37

#define ADXL345_FIFO_CTL 0x38		// FIFO control
#define ADXL345_FIFO_STATUS 0x39	// FIFO status

/* ------ other definations Pins,bits ------ */

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


/* 
 Interrupt PINs
 INT1: 0
 INT2: 1
 */
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/* 
 Interrupt bit position
 for registers 0x2E Interrupt Enable 0x2F Interrupt Map and 0x30 Interrupt source
 */
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

#define ADXL345_OK    1 // no error
#define ADXL345_ERROR 0 // indicates error is predent

#define ADXL345_NO_ERROR   0 // initial state
#define ADXL345_READ_ERROR 1 // problem reading accel
#define ADXL345_BAD_ARG    2 // bad method argument

class ADXL345
{
public:
  bool status;           // set when error occurs 
                         // see error code for details
  byte error_code;       // Initial state
  float gains[3];        // counts to Gs

  ADXL345();
  void init(int address);
  void powerOn();
  void readAccel(int* xyx);
  void readAccel(int* x, int* y, int* z);
  void get_Gxyz(float *xyz);

  void setTapThreshold(int tapThreshold);
  int getTapThreshold();
  void setAxisGains(float *_gains);
  void getAxisGains(float *_gains);
  void setAxisOffset(int x, int y, int z);
  void getAxisOffset(int* x, int* y, int*z);
  void setTapDuration(int tapDuration);
  int getTapDuration();
  void setDoubleTapLatency(int floatTapLatency);
  int getDoubleTapLatency();
  void setDoubleTapWindow(int floatTapWindow);
  int getDoubleTapWindow();
  void setActivityThreshold(int activityThreshold);
  int getActivityThreshold();
  void setInactivityThreshold(int inactivityThreshold);
  int getInactivityThreshold();
  void setTimeInactivity(int timeInactivity);
  int getTimeInactivity();
  void setFreeFallThreshold(int freeFallthreshold);
  int getFreeFallThreshold();
  void setFreeFallDuration(int freeFallDuration);
  int getFreeFallDuration();

  bool isActivityXEnabled();
  bool isActivityYEnabled();
  bool isActivityZEnabled();
  bool isInactivityXEnabled();
  bool isInactivityYEnabled();
  bool isInactivityZEnabled();
  bool isActivityAc();
  bool isInactivityAc();
  void setActivityAc(bool state);
  void setInactivityAc(bool state);

  bool getSuppressBit();
  void setSuppressBit(bool state);
  bool isTapDetectionOnX();
  void setTapDetectionOnX(bool state);
  bool isTapDetectionOnY();
  void setTapDetectionOnY(bool state);
  bool isTapDetectionOnZ();
  void setTapDetectionOnZ(bool state);

  void setActivityX(bool state);
  void setActivityY(bool state);
  void setActivityZ(bool state);
  void setInactivityX(bool state);
  void setInactivityY(bool state);
  void setInactivityZ(bool state);

  bool isActivitySourceOnX();
  bool isActivitySourceOnY();
  bool isActivitySourceOnZ();
  bool isTapSourceOnX();
  bool isTapSourceOnY();
  bool isTapSourceOnZ();
  bool isAsleep();

  bool isLowPower();
  void setLowPower(bool state);
  float getRate();
  void setRate(float rate);
  void set_bw(byte bw_code);
  byte get_bw_code();  

  byte getInterruptSource();
  bool getInterruptSource(byte interruptBit);
  bool getInterruptMapping(byte interruptBit);
  void setInterruptMapping(byte interruptBit, bool interruptPin);
  bool isInterruptEnabled(byte interruptBit);
  void setInterrupt(byte interruptBit, bool state);

  void getRangeSetting(byte* rangeSetting);
  void setRangeSetting(int val);
  bool getSelfTestBit();
  void setSelfTestBit(bool selfTestBit);
  bool getSpiBit();
  void setSpiBit(bool spiBit);
  bool getInterruptLevelBit();
  void setInterruptLevelBit(bool interruptLevelBit);
  bool getFullResBit();
  void setFullResBit(bool fullResBit);
  bool getJustifyBit();
  void setJustifyBit(bool justifyBit);
  void printAllRegister();
  void writeTo(byte address, byte val);

private:
  void readFrom(byte address, int num, byte buff[]);
  void setRegisterBit(byte regAdress, int bitPos, bool state);
  bool getRegisterBit(byte regAdress, int bitPos);  
  byte _buff[6] ;    //6 bytes buffer for saving data read from the device
  int _dev_address;
};
void print_byte(byte val);
#endif
