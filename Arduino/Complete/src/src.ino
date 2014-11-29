// Copyright (c) 2013 Nautilabs
// https://github.com/baptistelabat/robokite

// This file used SoftwareSerial example (Copyright (c) 2012 Dimension Engineering LLC)
// for Sabertooth http://www.dimensionengineering.com/software/SabertoothSimplifiedArduinoLibrary/html/index.html
// See license.txt in the Sabertooth arduino library for license details.
//
//
// Using example in http://code.google.com/p/arduino-pinchangeint/wiki/Usage
// Parts of code taken from http://playground.arduino.cc/Main/RotaryEncoders J.Carter(of Earth)
//
// The Sabertooth selector 1 3 5 6 have to be on on.

// Includes
#define NO_PORTB_PINCHANGES // PortB used for mySofwareSerial
#include <mySoftwareSerial.h> // This is a modified version of SoftwareSerial to be able to use interrupt pin as well
#include <SabertoothSimplified.h> // From http://www.dimensionengineering.com/software/SabertoothSimplifiedArduinoLibrary/html/_sabertooth_simplified_8h_source.html
#include <TinyGPS++.h>    // From https://github.com/mikalhart/TinyGPSPlus
#include <PID_v1.h>       // From https://github.com/br3ttb/Arduino-PID-Library/tree/master/PID_v1
#include <PinChangeInt.h> // From http://code.google.com/p/arduino-pinchangeint/

// Port definition
#define   POT_PIN  1 // Potentiometer
#define     A_PIN  2 // Linear encoder one
#define     B_PIN  3 // Linear encoder other one
#define RESET_PIN  4 // Linear encoder absolute reference
#define    TX_PIN  8 // Plug to Sabertooth RX
#define    RX_PIN 10 // Do not plug
#define   LED_PIN 13 // Built-in led
const int LINE_TENSION_PIN = A2; // For line tension measurement

// String management
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Sabertooth connection
// Note: NOT_A_PIN (0) was previously used for RX which is not used, but this was causing problems
mySoftwareSerial SWSerial(RX_PIN, TX_PIN); // RX, TX. RX on no pin (unused), TX on pin 8 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.
int power1, power2;

// NMEA protocol for robust messages
TinyGPSPlus nmea;
// O stands for Opensource, R for Robokite
// The index is the place of the field in the NMEA message
TinyGPSCustom feedback_request      (nmea, "ORFBR", 1);  // Feedback request
TinyGPSCustom pwm1     (nmea, "ORPW1", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 1
TinyGPSCustom pwm2     (nmea, "ORPW2", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 2
TinyGPSCustom setpos1  (nmea, "ORSP1", 1);  // Position setpoint for Sabertooth output 1
TinyGPSCustom setpos2  (nmea, "ORSP2", 1);  // Position setpoint for Sabertooth output 2
boolean isFeedbackRequested = false;

// PID for robust control
// Define Variables we'll be connecting to
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Input3;
// Specify the links and initial tuning parameters (Kp, Ki, Kd)
PID myPID1(&Input1, &Output1, &Setpoint1, 1, 0, 0, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, 1, 0, 0, DIRECT);

// Define global variables to deal with encoder interrupt
volatile unsigned long threshold = 1000;// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder. halfSteps
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;
// 'halfSteps' is the counter of half-steps. The actual
// number of steps will be equal to halfSteps / 2
volatile long halfSteps = 0;
long halfStepsCorrection = 0;
boolean isAbsoluteReferenceAvailable = false;

// Hardware specific parameters
// Potentiometer
#define POT_RANGE_DEG      300 // 300 is the value for standard potentiometer
#define POT_USED_RANGE_DEG  60 // To 
#define POT_OFFSET        0.05 // Distance from rotation axis to lever arm
#define NEUTRAL_ANGLE_DEG  225
// Linear encoder
#define LINEAR_RESOLUTION 0.005
#define LINEAR_USED_RANGE 0.05

void setup()
{
  pinMode(LED_PIN, OUTPUT); 
  
  // Initialize software serial communication with Sabertooth 
  SWSerial.begin(9600);
  
  // Initialize serial communications with computer
  Serial.begin(57600);
  
  // Reserve bytes for the inputString:
  inputString.reserve(200);
  
  // Initialize input and setpoint
  Input1 = 0;
  Input2 = 0;
  Input3 = 0;

  Setpoint1 = 0;
  Setpoint2 = 0;
  
  // Turn the PID on
  myPID1.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);
  myPID1.SetOutputLimits(-1, 1);
  myPID2.SetOutputLimits(-1, 1);
  
    //*****************************************************
  pinMode(A_PIN, INPUT);
  digitalWrite(A_PIN, HIGH);
  attachInterrupt(0, int0, CHANGE);
  //PCintPort::attachInterrupt(pinA, &int0, CHANGE);
  pinMode(B_PIN, INPUT);
  digitalWrite(B_PIN, HIGH);
  attachInterrupt(1, int1, CHANGE);
  //PCintPort::attachInterrupt(pinB, &int1, CHANGE);
  
  // Pin used to get absolute position
  pinMode(RESET_PIN, INPUT);
  digitalWrite(RESET_PIN, HIGH);
  PCintPort::attachInterrupt(RESET_PIN, &reset, CHANGE);
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
  // Print the string when a newline arrives:
  if (stringComplete)
  {   
    // Serial.println(inputString);
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
        if (setpos1.isUpdated())
        { 
          myPID1.SetMode(AUTOMATIC);
        }
        if (setpos2.isUpdated())
        {
          myPID2.SetMode(AUTOMATIC);
        }
        if (feedback_request.isUpdated())
        {
          isFeedbackRequested = true;
          digitalWrite(LED_PIN, HIGH);
        }    
      }
    }
    // Clear the string:
    inputString = "";
    stringComplete = false;
  }
}

#define POT_RANGE_DEG      300
#define POT_USED_RANGE_DEG  60
#define POT_OFFSET        0.05 //Distance from rotation axis to lever arm

// Linear encoder
#define LINEAR_RESOLUTION 0.005
#define LINEAR_USED_RANGE 0.05
void computeFeedback()
{
  
  // Feedbacks are normalized between -1 and 1
  
  // Potentiomete angle
  float rawAngle_deg = analogRead(POT_PIN)/1024.0*POT_RANGE_DEG;
  Input1 = (rawAngle_deg - NEUTRAL_ANGLE_DEG)/POT_USED_RANGE_DEG;
  
  // Linear encoder position
  Input2 = halfSteps*LINEAR_RESOLUTION/2/LINEAR_USED_RANGE;
  // Correct for lever arm (potentiometer not on axis)
  Input2+= (rawAngle_deg - NEUTRAL_ANGLE_DEG)* POT_OFFSET*3.14/180;
  
  // Line tension
  Input3 = analogRead(LINE_TENSION_PIN)/512 - 1;
}

void sendFeedback()
{
  if (isFeedbackRequested)
  {
    Serial.print((power1+128)*4);
    Serial.print(", ");
    Serial.print((power2+128)*4);
    Serial.print(", ");
    Serial.print((Input1+1)*512);
    Serial.print(", ");
    Serial.print((Input2+1)*512);
    Serial.print(", ");
    Serial.print((Input3+1)*512);
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
  if (myPID1.GetMode() == AUTOMATIC)
  {
    Setpoint1 = atoi(setpos1.value())/127.;
    myPID1.Compute();
    power1 = Output1*127;
  }
  if (myPID2.GetMode() == AUTOMATIC)
  {
    Setpoint2 = atoi(setpos2.value())/127.;
    Serial.println(Input2);
    myPID2.Compute();
    Serial.println(Output2);
    power2 = Output2*127;
  }
}

void sendOrder()
{
  // Order in the range -127/127
   ST.motor(1, power1);
   ST.motor(2, -power2);
}

void int0()
{
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND, A_PIN);
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
  int1signal = bitRead(PIND, B_PIN);
  if ( int1history==int1signal )
    return;
  int1time = micros();
}
void reset()
{
  halfStepsCorrection = halfSteps;
  isAbsoluteReferenceAvailable = true;
 }
 
