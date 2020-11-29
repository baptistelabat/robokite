
#include "makros.h"
#include "debug.h"
#include "RCReceive.h"

#define RC_CMD_PIN 7
#define RC_THRESHOLD_PIN 9

/*
  RC_Template.ino - Template for RC Receiver enabled programs interrupt version - Version 0.2
 Copyright (c) 2014 Wilfried Klaas.  All right reserved.
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
 
// uno 2, 3
//MEGA  2, 3, 21, 20, 19, 18
const byte PIN_RC_CH2 = 18;
const byte PIN_RC_CH4 = 19; 
const byte PIN_RC_CH5 = 20; 
const byte PIN_RC_CH6 = 21; 

# define RC_GND 2


RCReceive rcReceiver1;
RCReceive rcReceiver2;
RCReceive rcReceiver3;
RCReceive rcReceiver4;
RCReceive rcReceiver5;
RCReceive rcReceiver6;

byte value1, value2, value3, value4, value5, value6;

long RCIntegral1;

void setupRCReceiver() {
  // RC Receiver in Interruptvariante
  rcReceiver2.attachInt(PIN_RC_CH2);
  rcReceiver4.attachInt(PIN_RC_CH4);
  rcReceiver5.attachInt(PIN_RC_CH5);
  rcReceiver6.attachInt(PIN_RC_CH6);
    pinMode(RC_GND, OUTPUT);

    RCIntegral1 = 0;

  // put your setup code here, to run once:
}
void doWork()
{
  // put your main code here, to run repeatedly
  //value1 = rcReceiver1.getValue();
  value2 = rcReceiver2.getValue();
  //value3 = rcReceiver3.getValue();
  value4 = rcReceiver4.getValue();
  value5 = rcReceiver5.getValue();
  value6 = rcReceiver6.getValue();

  RCIntegral1 = RCIntegral1+ value1;

}
void loopRCReceiver() {
  if (
    (rcReceiver2.hasNP() && !rcReceiver2.hasError())
  &&(rcReceiver4.hasNP() && !rcReceiver4.hasError())
  &&(rcReceiver5.hasNP() && !rcReceiver5.hasError())
  &&(rcReceiver6.hasNP() && !rcReceiver6.hasError())
    )
  {
    doWork();
  } 
  else if (rcReceiver2.hasError()||rcReceiver4.hasError()||rcReceiver5.hasError()||rcReceiver6.hasError())
  {
    //doError();
  }
//  Serial.print(value1);
//  Serial.print("\t");
//  Serial.println(value2);
}


void doError() {
  // put your main code here, to run repeatedly
  value1 = 127;
  value2 = 0;
  value3 = 0;
}

byte getValue1()
{return value1;}
byte getValue2()
{return value2;}
byte getValue3()
{return value3;}
byte getValue4()
{return value4;}
byte getValue5()
{return value5;}
byte getValue6()
{return value6;}

long getRCIntegral1(){
  return RCIntegral1;
}
