
// Copyright (c) 2013 Nautilabs
// https://github.com/baptistelabat/robokite

// Using example in http://code.google.com/p/arduino-pinchangeint/wiki/Usage

#include <PinChangeInt.h>

//Parts of code taken from http://playground.arduino.cc/Main/RotaryEncoders J.Carter(of Earth)

// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder. 
//
// The threshold value I'm using limits it to 100 half pulses a second
//
// My encoder has 12 pulses per 360deg rotation and the specs say
// it is rated at a maximum of 100rpm.
//
// This threshold will permit my encoder to reach 250rpm so if it was connected
// to a motor instead of a manually operated knob I
// might possibly need to adjust it to 25000. However, this threshold
// value is working perfectly for my situation
//
volatile unsigned long threshold = 1000;

// 'rotaryHalfSteps' is the counter of half-steps. The actual
// number of steps will be equal to rotaryHalfSteps / 2
//
volatile long rotaryHalfSteps = 0;

// Working variables for the interrupt routines
//
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;

uint8_t latest_interrupted_pin;
uint8_t interrupt_count[20]={0}; // 20 possible arduino pins

int pinA = 2;
int pinB = 3;
int pinReset = 4;

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = 0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)


void int0()
	{
	if ( micros() - int0time < threshold )
		return;
	int0history = int0signal;
	int0signal = bitRead(PIND,pinA);
	if ( int0history==int0signal )
		return;
	int0time = micros();
	if ( int0signal == int1signal )
		rotaryHalfSteps++;
	else
		rotaryHalfSteps--;
	}

void int1()
	{
	if ( micros() - int1time < threshold )
		return;
	int1history = int1signal;
	int1signal = bitRead(PIND,pinB);
	if ( int1history==int1signal )
		return;
	int1time = micros();
	}
void reset()
{
 rotaryHalfSteps = 0;
 }

void setup() {
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
  Serial.begin(115200);
  Serial.println("---------------------------------------");
}

uint8_t i;

void loop() {
  long actualRotaryTicks = (rotaryHalfSteps / 2);
    // read the analog in value:
  Serial.print(rotaryHalfSteps*8+512); 
  Serial.print(",");  
  //Serial.print("\t output = ");      
  //Serial.println(outputValue);
  delay(2);
    
  for (int thisPin = 1; thisPin < 6; thisPin++) {
  sensorValue = analogRead(thisPin);            
  
  // print the results to the serial monitor:
  //Serial.print("sensor = " );                       
  Serial.print(sensorValue); 
  Serial.print(",");  
  //Serial.print("\t output = ");      
  //Serial.println(outputValue);
  delay(2);   
  }
   Serial.println("");
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);  
}















