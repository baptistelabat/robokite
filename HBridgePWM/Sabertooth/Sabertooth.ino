
// Copyright (c) 2013 Nautilabs
// https://github.com/baptistelabat/robokite

// This file used SoftwareSerial example (Copyright (c) 2012 Dimension Engineering LLC) for Sabertooth http://www.dimensionengineering.com/software/SabertoothSimplifiedArduinoLibrary/html/index.html
// See license.txt in the Sabertooth arduino library for license details.
// It is as well derived from HBridgePWM.ino from robokite project
// http://code.google.com/p/robokite/source/browse/HBridgePWM/HBridgePWM.ino
// The selector 1 3 5 6 have to be on on.
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <TinyGPS++.h>
#include <PID_v1.h>

SoftwareSerial SWSerial(NOT_A_PIN, 8); // RX on no pin (unused), TX on pin 11 (to S1).
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
TinyGPSCustom pwm1     (nmea, "ORPW1", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 1
TinyGPSCustom pwm2     (nmea, "ORPW2", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 2
TinyGPSCustom setss1   (nmea, "ORSS1", 1);  // Set speed for Sabertooth output 1
TinyGPSCustom setss2   (nmea, "ORSS2", 1);  // Set speed for Sabertooth output 2
TinyGPSCustom setpos1  (nmea, "ORSP1", 1);  // Position setpoint for Sabertooth output 1
TinyGPSCustom setpos2  (nmea, "ORSP2", 1);  // Position setpoint for Sabertooth output 2
TinyGPSCustom roll     (nmea, "ORKST", 1);  // Kite STate roll (rotation in camera frame)
TinyGPSCustom elevation(nmea, "ORKST", 2);  // Kite STate elevation
TinyGPSCustom bearing  (nmea, "ORKST", 3);  // Kite STate bearing 

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters (Kp, Ki, Kd)
PID myPID(&Input, &Output, &Setpoint, 10, 0, 1, DIRECT);

void setup()
{
  // Initialize software serial communication with Sabertooth 
  SWSerial.begin(9600);
  // Initialize serial communications at 9600 bps:
  Serial.begin(19200);
  Input = 0;
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-50, 50);
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
        alphaSigned1 = StrToFloat(pwm1.value());
        alphaSigned2 = StrToFloat(pwm2.value());
        Input = StrToFloat(elevation.value()); 
        lastSerialInputTime = millis();
      }
    }
  }  
}


void loop()
{
  myPID.Compute();
  int power1, power2;
  sensorValue = analogRead(analogInPin);
  setMode();
  //alphaSigned=Output/127.0;
  computeAlphaSigned();
  power1 = alphaSigned1*127;
  power2 = alphaSigned2*127;
  ST.motor(1, power1);
  ST.motor(2, power2);
  delay(10);
  if (millis()-lastWriteTime>100)
  {
    lastWriteTime = millis();
    
    Serial.print(power1);
    Serial.print(" ");
    Serial.println(power2);
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



