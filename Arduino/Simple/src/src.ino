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

const int ledPin = 13;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean isTimeOut = false;

//Note: NOT_A_PIN (0) was previously used for RX which is not used, but this was causing problems
SoftwareSerial SWSerial(10, 8); // RX, TX. RX on no pin (unused), TX on pin 8 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.
int power1, power2;
long lastSerialReception_ms;
long last_order_ms = 0;
#define ORDER_RATE_ms 100

TinyGPSPlus nmea;
// O stands for Opensource, R for Robokite
// The index is the place of the field in the NMEA message
TinyGPSCustom feedback_request      (nmea, "ORFBR", 1);  // Feedback request
TinyGPSCustom pwm1     (nmea, "ORPW1", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 1
TinyGPSCustom pwm2     (nmea, "ORPW2", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 2
TinyGPSCustom setpos1  (nmea, "ORSP1", 1);  // Position setpoint for Sabertooth output 1
TinyGPSCustom setpos2  (nmea, "ORSP2", 1);  // Position setpoint for Sabertooth output 2
boolean isFeedbackRequested = false;

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
    lastSerialReception_ms = millis();
    isTimeOut = false;
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
  if (fabs(millis()-lastSerialReception_ms)>1000)
  {
    isTimeOut = true;
  }
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
          pwm1.value();
        }
        if (pwm2.isUpdated())
        {
          pwm2.value();
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
  if (isTimeOut)
  {
    power1 = 0;
    power2 = 0;
  }
  else
  {
    power1 = atoi(pwm1.value());
    power2 = atoi(pwm2.value());
  }
    
}

void sendOrder()
{
  if (fabs(millis()-last_order_ms)>ORDER_RATE_ms)
  {
    last_order_ms = millis();
  // Order in the range -127/127
    //SWSerial.println(127);
      //Saturation based on posSat
   ST.motor(1, power1);
   ST.motor(2, power2);
}
}
 
