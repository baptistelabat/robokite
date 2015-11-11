/*
  Ultrasonic.cpp - Library for HC-SR04 Ultrasonic Ranging Module.library

  Created by ITead studio. Apr 20, 2010.
  iteadstudio.com
*/

#include "Arduino.h"
#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int TP, int EP)
{
   pinMode(TP,OUTPUT);
   pinMode(EP,INPUT);
   Trig_pin=TP;
   Echo_pin=EP;
}

double Ultrasonic::Timing(unsigned long timeOutus)
{
  digitalWrite(Trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_pin, LOW);
  duration = pulseIn(Echo_pin,HIGH, timeOutus);
  return duration;
}

double Ultrasonic::Ranging(int sys)
{
  unsigned long timeOutus;
  timeOutus = 50000;
  Timing(timeOutus);
  distance_cm = duration /29. / 2. ;
  distance_inc = duration / 74. / 2.;
  if (sys)
  return distance_cm;
  else
  return distance_inc;
}
