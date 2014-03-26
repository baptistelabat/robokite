
// Copyright (c) 2013 Nautilabs
// https://github.com/baptistelabat/robokite

// This file used SoftwareSerial example (Copyright (c) 2012 Dimension Engineering LLC) for Sabertooth http://www.dimensionengineering.com/software/SabertoothSimplifiedArduinoLibrary/html/index.html
// See license.txt in the Sabertooth arduino library for license details.
// It is as well derived from HBridgePWM.ino from robokite project
// https://github.com/baptistelabat/robokite/blob/master/HBridgePWM/HBridgePWM.ino
// The selector 1 3 5 6 have to be on on.
//
// Using example in http://code.google.com/p/arduino-pinchangeint/wiki/Usage
// Parts of code taken from http://playground.arduino.cc/Main/RotaryEncoders J.Carter(of Earth)

#define NO_PORTB_PINCHANGES // PortB used for mySofwareSerial
#include <mySoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <TinyGPS++.h>
#include <PID_v1.h>
#include <PinChangeInt.h>

mySoftwareSerial SWSerial(NOT_A_PIN, 8); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // Value read from the potentiometer
int initialSensorValue = 0;
double alphaSigned1 = 0;
double alphaSigned2 = 0;
String inputString = "";         // A string to hold incoming data
boolean stringComplete = false;  // Whether the string is complete
boolean isSerialControl = true; 
boolean isManualControl = false;
// Read the analog in value
// The range was reduced 
double sensorValueMin = 200;//0
double sensorValueMax = 800;//1023
double deadBand = 0.1; // Use to ensure zero speed. Should not be zero
double serialFrequency = 1;
long lastSerialInputTime = 0;
long lastWriteTime = 0;

TinyGPSPlus nmea;
// O stands for Opensource, R for Robokite
// The index is the place of the field in the NMEA message
TinyGPSCustom pwm1     (nmea, "ORPW1", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 1
TinyGPSCustom pwm2     (nmea, "ORPW2", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 2
TinyGPSCustom setss1   (nmea, "ORSS1", 1);  // Set speed for Sabertooth output 1
TinyGPSCustom setss2   (nmea, "ORSS2", 1);  // Set speed for Sabertooth output 2
TinyGPSCustom setpos1  (nmea, "ORSP1", 1);  // Position setpoint for Sabertooth output 1
TinyGPSCustom setpos2  (nmea, "ORSP2", 1);  // Position setpoint for Sabertooth output 2
TinyGPSCustom roll     (nmea, "ORKST", 1);  // Kite STate roll (rotation in camera frame)
TinyGPSCustom elevation(nmea, "ORKST", 2);  // Kite STate elevation
TinyGPSCustom bearing  (nmea, "ORKST", 3);  // Kite STate bearing
TinyGPSCustom kpm1      (nmea, "ORKP1", 1);  // Proportional coefficient multiplicator
TinyGPSCustom kim1      (nmea, "ORKI1", 1);  // Integral coefficient multiplicator
TinyGPSCustom kdm1      (nmea, "ORKD1", 1);  // Derivative coefficient multiplicator
TinyGPSCustom kpm2      (nmea, "ORKP2", 1);  // Proportional coefficient multiplicator
TinyGPSCustom kim2      (nmea, "ORKI2", 1);  // Integral coefficient multiplicator
TinyGPSCustom kdm2      (nmea, "ORKD2", 1);  // Derivative coefficient multiplicator
TinyGPSCustom kpmr      (nmea, "ORKPR", 1);  // Proportional coefficient multiplicator
TinyGPSCustom kimr      (nmea, "ORKIR", 1);  // Integral coefficient multiplicator
TinyGPSCustom kdmr      (nmea, "ORKDR", 1);  // Derivative coefficient multiplicator

// Define Variables we'll be connecting to
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double SetpointRoll, InputRoll, OutputRoll;

// Specify the links and initial tuning parameters (Kp, Ki, Kd)
PID myPID1(&Input1, &Output1, &Setpoint1, 1, 0, 0, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, 1, 0, 0, DIRECT);
PID myPIDRoll(&InputRoll, &OutputRoll, &SetpointRoll, 1, 0, 0, DIRECT);


//*****************************************************************************//
// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder. halfSteps
volatile unsigned long threshold = 1000;

// 'halfSteps' is the counter of half-steps. The actual
// number of steps will be equal to halfSteps / 2
//
volatile long halfSteps = 0;
long halfStepsCorrection = 0;

// Working variables for the interrupt routines
//
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;

uint8_t latest_interrupted_pin;
uint8_t interrupt_count[20] = {0}; // 20 possible arduino pins

int pinA = 2;
int pinB = 3;
int pinReset = 4;

boolean isAbsoluteReference = false;

// These constants won't change.  They're used to give names
// to the pins used:
const int potPin = 1;  // Analog input pin that the potentiometer is attached to

int barrePotentiometerValue = 0;        // value read from the pot

double linearResolution = 0.005; //Resolution of the linear encoder (in meters)
double linearRange = 0.05; //Range of the sensor to normalize data (+/- linearRange/2)

double pi = 3.1415;
double potentiometerRangeDeg = 300; 
double potentiometerMaxRange = potentiometerRangeDeg * pi/180;
double potentiometerUsedRange = pi/3;
double potentiometerDistance = 0.05; //Distance from rotation axis to lever arm
//*********************************************************************************************

void setup()
{
  // Initialize software serial communication with Sabertooth 
  SWSerial.begin(9600);
  
  // Initialize serial communications at 9600 bps:
  Serial.begin(19200);
  
  // Initialize input and setpoint
  Input1 = 0;
  Input2 = 0;
  InputRoll = 0;
  Setpoint1 = 0;
  Setpoint2 = 0;
  SetpointRoll = 0;
  
  //turn the PID on
  myPID1.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);
  myPIDRoll.SetMode(MANUAL);
  myPID1.SetOutputLimits(-1, 1);
  myPID2.SetOutputLimits(-1, 1);
  myPIDRoll.SetOutputLimits(-1, 1);
  
  //*****************************************************
  pinMode(pinA, INPUT);
  digitalWrite(pinA, HIGH);
  attachInterrupt(0, int0, CHANGE);
  //PCintPort::attachInterrupt(pinA, &int0, CHANGE);
  pinMode(pinB, INPUT);
  digitalWrite(pinB, HIGH);
  attachInterrupt(1, int1, CHANGE);
  //PCintPort::attachInterrupt(pinB, &int1, CHANGE);
  
  // Pin used to get absolute position
  pinMode(pinReset, INPUT);
  digitalWrite(pinReset, HIGH);
  PCintPort::attachInterrupt(pinReset, &reset, CHANGE);
  
}

void serialEvent() {

  inputString="";
  Serial.flush();
  while (Serial.available()) {
    // Get the new byte:
    char inChar = (char)Serial.read();
    // Add it to the inputString:
    inputString += inChar;
    // If the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  if (inputString[0] == 'i')
  {
    Serial.flush();
    isSerialControl = true;
    initialSensorValue = analogRead(analogInPin);// Read the joystick position to overide if joystick is moved
    if (inputString.substring(1)=="")
    {
      serialFrequency = 0;
    }
    else
    { 
      serialFrequency = StrToFloat(inputString.substring(1));
    } 
  }
  else
  {
    if (inputString == "o")
    {
      isSerialControl = false;
    }
    else
    {
      char c[50];
      inputString.toCharArray(c, 50);
      char *gpsStream = c;
      
      while (*gpsStream)
        if (nmea.encode(*gpsStream++))
      { 
        //Input1 = StrToFloat(elevation.value()); 
        lastSerialInputTime = millis();
        if (pwm1.isUpdated())
        { 
          myPID1.SetMode(MANUAL);
        }
        if (pwm2.isUpdated())
        {
          myPID2.SetMode(MANUAL);
        }
        if (setpos1.isUpdated())
        { 
          myPID1.SetMode(AUTOMATIC);
        }
        if (setpos2.isUpdated())
        {
          myPID2.SetMode(AUTOMATIC);
        }
          
      }
    }
  }  
}


void loop()
{
  
  int power1, power2;
  sensorValue = analogRead(analogInPin);

  process();
  
  power1 = -alphaSigned1*127;
  power2 = -alphaSigned2*127;
  ST.motor(1, power1);
  ST.motor(2, power2);
  delay(10);
  if (millis()-lastWriteTime>100)
  {
    lastWriteTime = millis();
    
    Serial.print(alphaSigned1);
    Serial.print(" ");
    Serial.println(alphaSigned2);
    if (isSerialControl)
    {
      //Serial.println("S");
    }
    else
    {
      if (isManualControl)
      {
        //Serial.println("M");
      }
    }

  }
  Serial.flush();
}

void setMode()
{  
  // If the analog value is changed, it means that the user wants to take control with the joystick
  // so fallback to joystick control
  if (fabs(initialSensorValue-sensorValue) > 10)
  {
    isSerialControl = false;
  }

  // Fallback to manual control if the analog voltage is close to zero (probably unplugged)
  // Place after hardware limit if possible
  // This avoids to go to full speed if one cable in the potentiometer is unplugged
  // Warning: if the midle cable of the potentiometer is unplugged, the signal is free, and the fault can not be detected
  // \todo invert polarity to be able to detect floating pin.
  isManualControl = false;//(sensorValue < 5)|(sensorValue >1023-5);
}

float StrToFloat(String str){
  char carray[str.length() + 1]; // Determine the size of the array
  str.toCharArray(carray, sizeof(carray)); // Put str into an array
  return atof(carray);
}

void computeAlphaSigned()
{
  if (serialFrequency != 0)
  { // Fallbacks to zero as we received no message in the expected time (twice the time)
    if ((millis()-lastSerialInputTime) > 2*1/serialFrequency*1000)
    {
      alphaSigned1 = 0;
      alphaSigned2 = 0;
    }
  }

  // If not in Serial control overwrite the value 
  if (false == isSerialControl)
  {
    // Compute the value corresponding to the deadband
    double sensorDeadBandMax = sensorValueMin + (deadBand/2.0+0.5)*(sensorValueMax - sensorValueMin);
    double sensorDeadBandMin = sensorValueMin + (-deadBand/2.0+0.5)*(sensorValueMax - sensorValueMin);
    if (sensorValue < sensorDeadBandMin) //strictly to avoid alphaSigned equals zero
    {
      alphaSigned1 = (sensorValue-sensorValueMin)/(sensorDeadBandMax - sensorValueMin)-1;
      alphaSigned2 = 0;
    } 
    else
    {
      if (sensorValue > sensorDeadBandMax)
      {
        alphaSigned1 = (sensorValue-sensorDeadBandMax)/(sensorValueMax - sensorDeadBandMax);
        alphaSigned2 = 0;
      }
      else
      {
        alphaSigned1 = 0;
        alphaSigned2 = 0;
      }
    }
    if (isManualControl)
    {
      alphaSigned1 = 0;
      alphaSigned2 = 0;
    }
  }
  alphaSigned1 = min(1, max(-1, alphaSigned1));
  alphaSigned2 = min(1, max(-1, alphaSigned2));
}


void int0()
{
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND, pinA);
  if ( int0history==int0signal )
    return;
  int0time = micros();
  if ( int0signal == int1signal )
    halfSteps++;
  else
    halfSteps--;
}

void int1()
{
  if ( micros() - int1time < threshold )
    return;
  int1history = int1signal;
  int1signal = bitRead(PIND, pinB);
  if ( int1history==int1signal )
    return;
  int1time = micros();
}

void reset()
{
  halfStepsCorrection = halfSteps;
  isAbsoluteReference = true;
 }
 
void computeFeedback()
{
  long steps = (halfSteps / 2);
  //Serial.print("T");
  //Serial.println(steps);
  double relative_position_m = (halfSteps)*linearResolution/2;
  double absolute_position_m = (halfSteps-halfStepsCorrection)*linearResolution/2;
  
  // read the analog in value:
  double raw_angle = analogRead(potPin)/1024.0*potentiometerMaxRange;
  double neutralAngle = 0.8*potentiometerMaxRange;
  double angle = raw_angle - neutralAngle;
  double correction_m = angle*potentiometerDistance;
  Input1 = angle/potentiometerUsedRange;
  Input2 = absolute_position_m/linearRange;
  //Serial.print("P");
  //Serial.println(Input1);
  //Serial.print("A");
  //Serial.println(Input2);
  
  InputRoll = StrToFloat(roll.value());

}

void process()
{
  //setMode();
  computeFeedback();
  if (myPID1.GetMode() ==MANUAL)
  {
    alphaSigned1 = StrToFloat(pwm1.value());
  }
  if (myPID1.GetMode() ==AUTOMATIC)
  {
    Setpoint1 = StrToFloat(setpos1.value());
    myPID1.Compute();
    alphaSigned1 = Output1;
  }
  if (myPID2.GetMode() ==MANUAL)
  {
    alphaSigned2 = StrToFloat(pwm2.value());
  }
  if (myPID2.GetMode() ==AUTOMATIC)
  {
    double offset_m = -0.02;
    Setpoint2 = StrToFloat(setpos2.value()) + offset_m/linearRange;
    myPID2.Compute();
    alphaSigned2 = Output2;
  }
}

void computePIDTuning()
{
  myPID1.SetTunings(StrToFloat(kpm1.value()), StrToFloat(kim1.value()), StrToFloat(kdm1.value()));
  myPID2.SetTunings(StrToFloat(kpm2.value()), StrToFloat(kim2.value()), StrToFloat(kdm2.value()));
  myPIDRoll.SetTunings(StrToFloat(kpmr.value()), StrToFloat(kimr.value()), StrToFloat(kdmr.value()));

}



