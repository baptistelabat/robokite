// Simple Example Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include "A_RCReceiver.h"
#include "B_HX711.h"
#include "C_Sabertooth.h"
#include "D_Servo.h"






void setup()
{
  Serial.begin(57600);
  setupSabertooth();
  setupRCReceiver();
  setupHX711();
  setupServo();
}


void loop()
{
  loopHX711();
  int cmd, threshold, CLcmd, maxcmd, servocmd;
  double lineTension, CL;
  lineTension = getLineTension()-50;
  loopRCReceiver();
  threshold = getValue3();
  cmd = getValue1()-127;
  maxcmd = getValue2()-127;
  maxcmd = constrain(maxcmd, 0, 127);
  CL = maxcmd-threshold/255.0*maxcmd*lineTension/1500.0*10;
  CLcmd = constrain(CL, 0, maxcmd);
  cmd = cmd  - CLcmd;
  servocmd = 120+60*sin(get_cmd_integral()/127.0);
  Serial.print(lineTension);
  Serial.print('\t');
  Serial.print(threshold);
  Serial.print('\t');
  Serial.println(CL);
  Serial.print(servocmd);
  Serial.println(cmd);
  sendSabertooth(cmd);
  
  sendServo(servocmd);
}
