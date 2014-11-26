// Copyright (c) 2013 Nautilabs
// https://github.com/baptistelabat/robokite

// This file used SoftwareSerial example (Copyright (c) 2012 Dimension Engineering LLC)
// for Sabertooth http://www.dimensionengineering.com/software/SabertoothSimplifiedArduinoLibrary/html/index.html
// See license.txt in the Sabertooth arduino library for license details.
//
// The selector 1 3 5 6 have to be on on.
//
// Using example in http://code.google.com/p/arduino-pinchangeint/wiki/Usage
// Parts of code taken from http://playground.arduino.cc/Main/RotaryEncoders J.Carter(of Earth)
//
//
// Connections
// Arduino          Sabertooth
// 8 (soft RX)      TX


#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <TinyGPS++.h>
#include <PID_v1.h>

const int ledPin = 13;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

//Note: NOT_A_PIN (0) was previously used for RX which is not used, but this was causing problems
SoftwareSerial SWSerial(10, 8); // RX, TX. RX on no pin (unused), TX on pin 8 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.
int power1, power2;

TinyGPSPlus nmea;
// O stands for Opensource, R for Robokite
// The index is the place of the field in the NMEA message
TinyGPSCustom feedback_request      (nmea, "ORFBR", 1);  // Feedback request
TinyGPSCustom pwm1     (nmea, "ORPW1", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 1
TinyGPSCustom pwm2     (nmea, "ORPW2", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 2
TinyGPSCustom setpos1  (nmea, "ORSP1", 1);  // Position setpoint for Sabertooth output 1
TinyGPSCustom setpos2  (nmea, "ORSP2", 1);  // Position setpoint for Sabertooth output 2
boolean isFeedbackRequested = false;

// Define Variables we'll be connecting to
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
// Specify the links and initial tuning parameters (Kp, Ki, Kd)
PID myPID1(&Input1, &Output1, &Setpoint1, 1, 0, 0, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, 1, 0, 0, DIRECT);

void setup()
{
  // Initialize software serial communication with Sabertooth 
  SWSerial.begin(9600);
  
  // Initialize serial communications with computer
  Serial.begin(57600);
  
  // reserve bytes for the inputString:
  inputString.reserve(200);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent()
{
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}

void loop()
{
   processSerialInput();
   computeFeedback();
   sendFeedback();
   computeOrder();
   sendOrder();
   delay(10);
}

void processSerialInput()
{
  // print the string when a newline arrives:
  if (stringComplete)
  {   
    //Serial.println(inputString);
    char ctab[inputString.length()+1];
    inputString.toCharArray(ctab, sizeof(ctab));
    char *gpsStream = ctab;
    
    while (*gpsStream)
    {
      if (nmea.encode(*gpsStream++))
      {
        if (pwm1.isUpdated())
        { 
          myPID1.SetMode(MANUAL);
        }
        if (pwm2.isUpdated())
        {
          myPID2.SetMode(MANUAL);
        }
        if (feedback_request.isUpdated())
        {
          isFeedbackRequested = true;
          digitalWrite(ledPin, HIGH);
        }
        
      }
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

void computeFeedback()
{
}

void sendFeedback()
{
  if (isFeedbackRequested)
  {
    Serial.print((power1+127)*4);
    Serial.print(", ");
    Serial.print((power2+127)*4);
    Serial.println("");
    isFeedbackRequested = false;
  }
}

void computeOrder()
{
  if (myPID1.GetMode() == MANUAL)
  {
    power1 = atoi(pwm1.value());
  }
  if (myPID2.GetMode() == MANUAL)
  {
    power2 = atoi(pwm2.value());
  }
}

void sendOrder()
{
  // Order in the range -127/127
   ST.motor(1, power1);
   ST.motor(2, power2);
}
 
