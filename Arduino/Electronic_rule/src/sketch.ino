
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
volatile unsigned long threshold = 10000;

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

void int0()
	{
	if ( micros() - int0time < threshold )
		return;
	int0history = int0signal;
	int0signal = bitRead(PIND,2);
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
	int1signal = bitRead(PIND,3);
	if ( int1history==int1signal )
		return;
	int1time = micros();
	}
void reset()
{
 rotaryHaflSteps = 0;
 }

void setup() {
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);
  PCintPort::attachInterrupt(2, &int0, CHANGE);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);
  PCintPort::attachInterrupt(3, &int1, CHANGE);
  
  // Pin used to get absolute position
  pinMode(4, INPUT);
  digitalWrite(4, HIGH);
  PCintPort::attachInterrupt(4, &reset, CHANGE);
  Serial.begin(115200);
  Serial.println("---------------------------------------");
}

uint8_t i;

void loop() {
  long actualRotaryTicks = (rotaryHalfSteps / 2);
  Serial.println(actualRotaryTicks, DEC);
}















