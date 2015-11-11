/*
  Ultrasonic.h - Library for HR-SC04 Ultrasonic Ranging Module.
  Created by ITead studio. Alex, Apr 20, 2010.
  iteadstudio.com
*/


#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

#define CM 1
#define INC 0

class Ultrasonic
{
  public:
    Ultrasonic(int TP, int EP);
    double Timing(unsigned long timeOutus);
    double Ranging(int sys);

    private:
    int Trig_pin;
    int Echo_pin;
    double  duration,distance_cm,distance_inc;
    
};

#endif
