
/*---- Instructions

1.To run this code you will need to install the pinchangeint library
   Link - http://code.google.com/p/arduino-pinchangeint/wiki/Installation

2. The 5 pins are named- AILERON, ELEVATION, THROTTLE, RUDDER, POWER

3. You may choose to read the long explanations in the comments & links which explain the precautions in giving inputs to the servo motors

4. The following code provides simple pass through  this is a good initial test,
 the Arduino will pass through receiver input as if the Arduino is not there.
  This should be used to confirm the circuit and power before attempting any custom processing in a project.
  We have generated angle outputs using receiver and will input them to PID and run motors using final processed values
*/

//------------------------Reading Material------------------------------------------------------------
//
// You can refer to related posts - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
//-----------------------------------------------------------------------------------------------------



#include <PinChangeInt.h>

#include <Servo.h>

// Assign your channel input pins

#define AILERON_IN_PIN 8
#define ELEVATION_IN_PIN 9
#define THROTTLE_IN_PIN 11
#define RUDDER_IN_PIN 10
#define POWER_IN_PIN 12

// Assign your channel output pins
#define AILERON_OUT_PIN 3
#define ELEVATION_OUT_PIN 4
#define THROTTLE_OUT_PIN 5
#define RUDDER_OUT_PIN 6
#define POWER_OUT_PIN 7


// These BIT flags are set in bUpdateFlagsShared to indicate which channels have received new signals
#define AILERON_FLAG 1
#define ELEVATION_FLAG 2
#define THROTTLE_FLAG 4
#define RUDDER_FLAG 8
#define POWER_FLAG 16

// Servo objects generate the signals expected by "Electronic Speed Controllers(ESC)" and Servos
Servo servoAileron;
Servo servoElevation;
Servo servoThrottle;
Servo servoRudder;
Servo servoPower;

// update flags (updated by the ISR and read by loop.)
volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unAileronInShared;
volatile uint16_t unElevationInShared; 
volatile uint16_t unThrottleInShared;        
volatile uint16_t unRudderInShared;
volatile uint16_t unPowerInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR but if we need refer to these in loop and the ISR then use volatile 
uint32_t ulAileronStart;
uint32_t ulElevationStart;
uint32_t ulThrottleStart;
uint32_t ulRudderStart;
uint32_t ulPowerStart;

void setup()
{
  Serial.begin(9600);
  
  Serial.println("multiChannels");
  
  
  // attach servo objects
  servoAileron.attach(AILERON_OUT_PIN);
  servoElevation.attach(ELEVATION_OUT_PIN);
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoRudder.attach(RUDDER_OUT_PIN);
  servoPower.attach(POWER_OUT_PIN);
  
  // Using the PinChangeInt library, attach the interrupts used to read the channels
  PCintPort::attachInterrupt(AILERON_IN_PIN, calcAileron,CHANGE); 
  PCintPort::attachInterrupt(ELEVATION_IN_PIN, calcElevation,CHANGE); 
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(RUDDER_IN_PIN, calcRudder,CHANGE); 
  PCintPort::attachInterrupt(POWER_IN_PIN, calcPower,CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that their values will be retained 
  // between calls to loop.
  static uint16_t unAileronIn;
  static uint16_t unElevationIn;
  static uint16_t unThrottleIn;
  static uint16_t unRudderIn;
  static uint16_t unPowerIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    
// In loop we immediately take local copies so that the ISR can keep ownership of the 
// shared ones. During copying, we need to turn the interrupts off and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals

    bUpdateFlags = bUpdateFlagsShared;
    
    // only copy when the appropriate flags are high....
    
    if(bUpdateFlags & AILERON_FLAG)
    {
      unAileronIn = unAileronInShared;
    }
    
    if(bUpdateFlags & ELEVATION_FLAG)
    {
      unElevationIn = unElevationInShared;
      
    }
    
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
    
    if(bUpdateFlags & RUDDER_FLAG)
    {
      unRudderIn = unRudderInShared;
    }
    
    if(bUpdateFlags & POWER_FLAG)
    {
      unPowerIn = unPowerInShared;
    }
     
    // clear 'shared' copy of updated flags as we have already taken the updates
    bUpdateFlagsShared = 0;
    
    interrupts();
  }
  
  // Processing starts from here onwards
  

/* Servo and Conversion Info 
  
1. Servo output can be (0,180)degrees or in microseconds
   On standard servos a parameter value of 1000us is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle
   For imperfect ones try values between 700us and 2300us

2. I

3. For conversion, we can use the mapping function
   val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)   [I/O conversion]
   In this code I have commented out the angle conversions which can be done by testing out the maximum and minimum values
   ... I have directly tried to run the motors using the input values
*/

  if(bUpdateFlags & AILERON_FLAG)
  {
//     unAileronIn = map(unAileronIn, 1060, 2060, 0, 179);
      if(servoAileron.readMicroseconds() != unAileronIn)
    {
      servoAileron.writeMicroseconds(unAileronIn);         
    }
     
        														
  }

  if(bUpdateFlags & ELEVATION_FLAG)
  {
//      unElevationIn = map(unElevationIn, 1215, 2255, 0, 179);	
       if(servoElevation.readMicroseconds() != unElevationIn)
    {
      servoElevation.writeMicroseconds(unElevationIn);         
    }
  }
  
  if(bUpdateFlags & THROTTLE_FLAG)
  {
//        unThrottleIn = map(unThrottleIn, 940, 2000, 0, 179);
	if(servoThrottle.readMicroseconds() != unThrottleIn)
    {
      servoThrottle.writeMicroseconds(unThrottleIn);         
    }
  }
  
  if(bUpdateFlags & RUDDER_FLAG)
  {
//        unRudderIn = map(unRudderIn, 950, 2020, 0, 179);
         if(servoRudder.readMicroseconds() != unRudderIn)
    {
      servoRudder.writeMicroseconds(unRudderIn);         
    }
  }
  
    if(bUpdateFlags & POWER_FLAG)
  {
//      unPowerIn = map(unPowerIn, 930, 2030, 0, 179);
      if(servoPower.readMicroseconds() != unPowerIn)
    {
      servoPower.writeMicroseconds(unPowerIn);         
    }
  }
  bUpdateFlags = 0;
}


// Interrupt service routine (ISR) functions below:

void calcAileron()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(AILERON_IN_PIN) == HIGH)
  { 
    ulAileronStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unAileronInShared = (uint16_t)(micros() - ulAileronStart);
    // use set the throttle flag to indicate that a new throttle signal has been received, similar mechanism is used for codes below
    bUpdateFlagsShared |= AILERON_FLAG;
  }
}

void calcElevation()
{
  if(digitalRead(ELEVATION_IN_PIN) == HIGH)
  { 
    ulElevationStart = micros();
  }
  else
  {
    unElevationInShared = (uint16_t)(micros() - ulElevationStart);
    bUpdateFlagsShared |= ELEVATION_FLAG;
  }
}

void calcThrottle()
{
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcRudder()
{
  if(digitalRead(RUDDER_IN_PIN) == HIGH)
  { 
    ulRudderStart = micros();
  }
  else
  {
    unRudderInShared = (uint16_t)(micros() - ulRudderStart);
    bUpdateFlagsShared |= RUDDER_FLAG;
  }
}

void calcPower()
{
  if(digitalRead(POWER_IN_PIN) == HIGH)
  { 
    ulPowerStart = micros();
  }
  else
  {
    unPowerInShared = (uint16_t)(micros() - ulPowerStart);
    bUpdateFlagsShared |= POWER_FLAG;
  }
}
