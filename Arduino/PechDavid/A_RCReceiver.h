
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
const byte PIN_RC = 19; 
const byte PIN_RC1 = 20; 

const byte PIN_RC2 = 21; 


RCReceive rcReceiver1;
RCReceive rcReceiver2;
RCReceive rcReceiver3;

byte value1;
byte value2;
byte value3;

void setupRCReceiver() {
  // RC Receiver in Interruptvariante
  rcReceiver1.attachInt(PIN_RC);
  rcReceiver2.attachInt(PIN_RC1);
  rcReceiver3.attachInt(PIN_RC2);

  // put your setup code here, to run once:
}
void doWork()
{
  // put your main code here, to run repeatedly
  value1 = rcReceiver1.getValue();
  value2 = rcReceiver2.getValue();
  value3 = rcReceiver3.getValue();

}
void loopRCReceiver() {
  if ((rcReceiver1.hasNP() && !rcReceiver1.hasError())&&(rcReceiver2.hasNP() && !rcReceiver2.hasError())&&(rcReceiver3.hasNP() && !rcReceiver3.hasError()))
  {
    doWork();
  } 
  else if (rcReceiver1.hasError()||rcReceiver2.hasError()||rcReceiver3.hasError())
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
