// Copyright (c) 2013 Nautilabs
// https://github.com/baptistelabat/robokite

// This file used SoftwareSerial example (Copyright (c) 2012 Dimension Engineering LLC)
// for Sabertooth http://www.dimensionengineering.com/software/SabertoothSimplifiedArduinoLibrary/html/index.html
// See license.txt in the Sabertooth arduino library for license details.
//
// The Sabertooth selector 1 3 5 6 have to be on on.

// Includes
#include <SoftwareSerial.h> 
#include <SabertoothSimplified.h> // From http://www.dimensionengineering.com/software/SabertoothArduinoLibraries.zip
#include <TinyGPS++.h>    // From https://github.com/mikalhart/TinyGPSPlus
#include <PID_v1.h>       // From https://github.com/br3ttb/Arduino-PID-Library/tree/master/PID_v1
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile
#include <MavlinkForArduino.h>

// Port definition
#define      TX_PIN 8 // Plug to Sabertooth RX
#define      RX_PIN 10 // Do not plug
#define RF_DATA_PIN 11
#define     LED_PIN 13 // Built-in led


// String management
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Sabertooth connection
// Note: NOT_A_PIN (0) was previously used for RX which is not used, but this was causing problems
SoftwareSerial SWSerial(RX_PIN, TX_PIN); // RX, TX. RX on no pin (unused), TX on pin 8 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.
int power1, power2;

// NMEA protocol for robust messages
TinyGPSPlus nmea;
// O stands for Opensource, R for Robokite
// The index is the place of the field in the NMEA message
TinyGPSCustom feedback_request      (nmea, "ORFBR", 1);  // Feedback request. The feedback are normalized between 0 and 1023
// The orders are normalized between -127 and 127
TinyGPSCustom pwm1     (nmea, "ORPW1", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 1
TinyGPSCustom pwm2     (nmea, "ORPW2", 1);  // Dimentionless voltage setpoint (Pulse Width Modulation) for Sabertooth output 2
TinyGPSCustom setpos1  (nmea, "ORSP1", 1);  // Position setpoint for Sabertooth output 1
TinyGPSCustom setpos2  (nmea, "ORSP2", 1);  // Position setpoint for Sabertooth output 2
TinyGPSCustom kpm1 (nmea, "ORKP1", 1); // Proportional coefficient multiplicator
TinyGPSCustom kim1 (nmea, "ORKI1", 1); // Integral coefficient multiplicator
TinyGPSCustom kdm1 (nmea, "ORKD1", 1); // Derivative coefficient multiplicator
TinyGPSCustom kpm2 (nmea, "ORKP2", 1); // Proportional coefficient multiplicator
TinyGPSCustom kim2 (nmea, "ORKI2", 1); // Integral coefficient multiplicator
TinyGPSCustom kdm2 (nmea, "ORKD2", 1); // Derivative coefficient multiplicator
boolean isFeedbackRequested = false;

uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
uint8_t buflen = sizeof(buf);
uint8_t data[4];  // 2 element array of unsigned 8-bit type, holding Joystick readings
RH_ASK driver(4800, RF_DATA_PIN, 6);
boolean isRFUpdated = false;
// RECEPTEUR : DATA D11

// PID for robust control
// Define Variables we'll be connecting to
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Input3;
// Specify the links and initial tuning parameters (Kp, Ki, Kd)
float Kp1 = 1;
float Ki1 = 0.001;
float Kd1 = 0.001;
float Kp2 = 1;
float Ki2 = 0.001;
float Kd2 = 0.001;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

// Hardware specific parameters
// Potentiometer
#define POT_RANGE_DEG      300 // 300 is the value for standard potentiometer
#define POT_USED_RANGE_DEG  60 // To normalize and saturate rotation
#define POT_OFFSET        0.05 // Distance from rotation axis to lever arm (in m)
#define NEUTRAL_ANGLE_DEG    90 // Zero of the potentiometer
// Linear encoder
#define LINEAR_RESOLUTION 0.005// Resolution of the linear encoder
#define LINEAR_USED_RANGE 0.05 // To normalize and saturate translation motion
#define PI 3.1415

#define ORDER_RATE_ms 100
long last_order_ms = 0;
void setup()
{
  pinMode(LED_PIN, OUTPUT); 
  
  // Initialize software serial communication with Sabertooth 
  SWSerial.begin(9600);
  
  // Initialize serial communications with computer
  Serial.begin(57600);
  
  if (!driver.init())
    Serial.println("init failed");
  
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
  // Saturate the output to the maximum range (values are normalized between -1 and 1)
  myPID1.SetOutputLimits(-1, 1);
  myPID2.SetOutputLimits(-1, 1);
  
  for (int i=0;i<4;i++)
  {
    data[i] = 127;
  }
  data[0] = NEUTRAL_ANGLE_DEG*255./POT_RANGE_DEG+127;
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
   processRFInput();
   computeFeedback();
   sendFeedback();
   computeOrder();
   sendOrder();
   delay(10);
}
void processRFInput()
{
    if (driver.recv(buf, &buflen)) // Non-blocking
    {
	// Message with a good checksum received, dump it.
	//driver.printBuffer("Got:", buf, buflen);
        for (byte i = 0; i < buflen; i++) // Si il n'est pas corrompu on l'affiche via Serial
	{ 
          data[i] = buf[i];
        }
        isRFUpdated = true;       
    }
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
          // $ORFBR,0*57
          isFeedbackRequested = true;
          digitalWrite(LED_PIN, HIGH);
        }
        if (kpm1.isUpdated()||kim1.isUpdated()||kdm1.isUpdated()||kpm2.isUpdated()||kim2.isUpdated()||kdm2.isUpdated())
        {
          computePIDTuning();
        }   
      }
    }
    // Clear the string:
    inputString = "";
    stringComplete = false;
  }
}

void computeFeedback()
{
  
  // Feedbacks are normalized between -1 and 1
  
  // Potentiometer angle
  float rawAngle_deg = (data[0]-127)/255.0*POT_RANGE_DEG;
  Input1 = (rawAngle_deg - NEUTRAL_ANGLE_DEG)*1.0/POT_USED_RANGE_DEG;
  
  // Linear encoder position
  Input2 = (data[1]-127);//*LINEAR_RESOLUTION/2./LINEAR_USED_RANGE;
  // Correction for lever arm (potentiometer not on axis)
  //Input2+= (rawAngle_deg - NEUTRAL_ANGLE_DEG)* POT_OFFSET*PI/180;
  
  // Line tension
  Input3 = data[3]/127. - 1;
}

void sendFeedback()
{
  mavlink_message_t msg; 
  uint8_t bufout[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  uint64_t time_us = micros();
  int group_mlx = 0;
  uint8_t system_id = 100;
  uint8_t target_system = system_id;
  uint8_t component_id = 1;
  uint8_t target_component = 2;// Sabertooth
  
  // All the feedback values are normalized in the range 0-1023 (10 bits resolution)
  if (isFeedbackRequested)
  {
    
    if (isRFUpdated)
    {
      float fdbk[8];//fdbk
      fdbk[0] = float(Input1);
      fdbk[1] = float(Input2);
      fdbk[2] = float(Input3);

      isRFUpdated = false;

      //mavlink_msg_actuator_control_target_pack(system_id, component_id, &msg, time_us, group_mlx, fdbk);
      //len = mavlink_msg_to_send_buffer(bufout, &msg);
      //Serial.write(bufout, len);
    }
    float orders[8];
    orders[0] = float(power1/127.);
    orders[1] = float(power2/127.);
    //mavlink_msg_set_actuator_control_target_pack(system_id, component_id, &msg, time_us, group_mlx, target_system, target_component, orders);
    //len = mavlink_msg_to_send_buffer(bufout, &msg);
    //Serial.write(bufout, len);
    
    Serial.print((power1+127)*4);
    Serial.print(", ");
    Serial.print((power2+127)*4);
    Serial.print(", ");
    Serial.print((Input1+1)/2.*1023);
    Serial.print(", ");
    Serial.print((Input2/127.+1)/2.*1023);
    Serial.print(", ");
    Serial.print((Input3+1)/2.*1023);
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
    myPID2.Compute();
    power2 = Output2*127;
  }
}

void sendOrder()
{
  if (fabs(millis()-last_order_ms)>ORDER_RATE_ms)
  {
    last_order_ms = millis();
  // Order in the range -127/127
    //SWSerial.println(127);
   ST.motor(1, power1);
   ST.motor(2, power2);
  }
}
void computePIDTuning()
{
 myPID1.SetTunings(atof(kpm1.value())*Kp1, atof(kim1.value())*Ki1, atof(kdm1.value())*Kd1);
 myPID2.SetTunings(atof(kpm2.value())*Kp2, atof(kim2.value())*Ki2, atof(kdm2.value())*Kd2);
}
 
