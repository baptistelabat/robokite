// ask_transmitter.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to transmit messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) transmitter with an TX-C1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

// Includes
#define NO_PORTB_PINCHANGES // PortB used for mySofwareSerial
#include <PinChangeInt.h> // From https://github.com/GreyGnome/PinChangeInt

// Port definition

#define         A_PIN  2 // Linear encoder one
#define         B_PIN  3 // Linear encoder other one
#define     RESET_PIN  4 // Linear encoder absolute reference
#define  DATA_OUT_PIN  5 // RF module ATAD pin
#define   LED_PIN 13 // Built-in led
const int   POT_PIN = A7; // Potentiometer
const int LINE_TENSION_PIN = A6; // For line tension measurement


RH_ASK driver(4800, 11, DATA_OUT_PIN);
//Transmitter : ATAD D12

uint8_t data[4];  // 2 element array of unsigned 8-bit type, holding Joystick readings

// Define global variables to deal with encoder interrupt
volatile unsigned long threshold = 1000;// 'threshold' is the De-bounce Adjustment factor for the encoder. halfSteps
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;
// 'halfSteps' is the counter of half-steps. The actual
// number of steps will be equal to halfSteps / 2
volatile long halfSteps = 0;
long halfStepsCorrection = -127; //to get out of range value
boolean isAbsoluteReferenceAvailable = false;

#define FEEDBACK_RATE_ms 50
long last_sent_ms = 0;

void setup()
{
    Serial.begin(9600);	  // Debugging only
    if (!driver.init())
         Serial.println("init failed");
    data[0] = 127;
    data[1] = 127;
    data[2] = 0;
    data[3] = 127;
 
   // Prepare interrupts for the linear encoder
  pinMode(A_PIN, INPUT);
  digitalWrite(A_PIN, HIGH);
  attachInterrupt(0, int0, CHANGE);

  pinMode(B_PIN, INPUT);
  digitalWrite(B_PIN, HIGH);
  attachInterrupt(1, int1, CHANGE);
  
  // Pin used to get absolute position
  pinMode(RESET_PIN, INPUT);
  digitalWrite(RESET_PIN, HIGH);
  PCintPort::attachInterrupt(RESET_PIN, &reset, CHANGE);   
    
}

void loop()
{
    computeFeedback();
    sendFeedback();
    delay(10);
}

void computeFeedback()
{
  
  // Feedbacks are normalized between -1 and 1
  
  // Potentiometer angle
  data[0] = map(analogRead(POT_PIN), 0, 1023, 0, 255);
  data[1] = map(halfSteps + 127, 0, 255, 0, 255); //Map for clean saturation
  if (isAbsoluteReferenceAvailable)
  {
    data[2] = map(halfStepsCorrection + 127, 0, 255, 0, 255); //Map for clean saturation
  }
  else
  {
    data[2] = 0;
  }
  data[3] = map(analogRead(LINE_TENSION_PIN), 0, 1023, 0, 255);

}
void sendFeedback()
{
  // All the feedback values are normalized in the range 0-1023 (10 bits resolution)
  if (fabs(millis()-last_sent_ms)>FEEDBACK_RATE_ms)
  {
    driver.send(data, sizeof(data));
    driver.waitPacketSent();
  }
}
// Taken from http://playground.arduino.cc/Main/RotaryEncoders J.Carter(of Earth)
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

// Taken from http://playground.arduino.cc/Main/RotaryEncoders J.Carter(of Earth)
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
 
