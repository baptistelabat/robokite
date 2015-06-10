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

#define  DATA_OUT_PIN  2 // RF module ATAD pin
#define   LED_PIN 13 // Built-in led
#define   PIN_5V_1 12
#define   PIN_0V_1 11
#define   PIN_5V_2 9
#define   PIN_0V_2 10
const int   POT_PIN1 = A7; // Potentiometer
const int   POT_PIN2 = A6; // Potentiometer
const int LINE_TENSION_PIN1 = A5; // For line tension measurement
const int LINE_TENSION_PIN2 = A4; // For line tension measurement


RH_ASK driver(4800, 11, DATA_OUT_PIN);
//Transmitter : ATAD D12

uint8_t data[4];  // 2 element array of unsigned 8-bit type, holding Joystick readings

#define FEEDBACK_RATE_ms 50
long last_sent_ms = 0;

#define POT_RANGE_DEG      300 // 300 is the value for standard potentiometer


void setup()
{
    Serial.begin(9600);	  // Debugging only
    if (!driver.init())
         Serial.println("init failed");
    data[0] = 127;
    data[1] = 127;
    data[2] = 0;
    data[3] = 127;  
    
    pinMode(PIN_0V_1, OUTPUT);
    pinMode(PIN_0V_2, OUTPUT);
    pinMode(PIN_5V_1, OUTPUT);
    pinMode(PIN_5V_2, OUTPUT);    
}

void loop()
{   
  digitalWrite(PIN_5V_1, HIGH);
  digitalWrite(PIN_5V_2, HIGH);
  digitalWrite(PIN_0V_1, LOW);
  digitalWrite(PIN_0V_2, LOW);
    sendFeedback();
    delay(10);
}

void computeFeedback()
{
  
  // Feedbacks are normalized between -1 and 1
  
  // Potentiometer angle
  data[0] = map(analogRead(POT_PIN1),(POT_RANGE_DEG-180)/300*1023, 1023, 0, 255);
  data[1] = map(analogRead(POT_PIN2),(POT_RANGE_DEG-180)/300*1023, 1023, 0, 255); //Map for clean saturation
  data[2] = map(analogRead(LINE_TENSION_PIN1), 0, 1023, 0, 255);
  data[3] = map(analogRead(LINE_TENSION_PIN2), 0, 1023, 0, 255);

}
void sendFeedback()
{
  // All the feedback values are normalized in the range 0-1023 (10 bits resolution)
  if (fabs(millis()-last_sent_ms)>FEEDBACK_RATE_ms)
  {
    last_sent_ms = millis();
    computeFeedback();
    driver.send(data, sizeof(data));
    driver.waitPacketSent();
    Serial.print(data[0]);
    Serial.print(", ");
        Serial.print(data[1]);
    Serial.print(", ");
        Serial.print(data[2]);
    Serial.print(", ");
        Serial.print(data[3]);
    Serial.println(", ");
  }
}
