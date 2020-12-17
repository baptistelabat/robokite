// Simple Example Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include "A_RCReceiver.h"
#include "B_HX711.h"
#include "C_Sabertooth.h"
#include "D_Servo.h"

double lineTension_offset;




void setup()
{
  Serial.begin(57600);
  setupSabertooth();
  setupRCReceiver();
  setupHX711();
  setupServo();
  lineTension_offset =0;
}


void loop()
{
  loopHX711();
  int cmd, manucmd, threshold_min,threshold_max, CLcmd, maxcmd, servocmd, gain;
  double lineTension, CL;
  lineTension = getLineTension();
  loopRCReceiver();
  threshold_min = getValue5()-30;
  manucmd = getValue2()-127;
  threshold_max = 20*(getValue6()-30);
  if (threshold_max < threshold_min +10)
  {threshold_max = threshold_min+10;}
  gain = getValue4()-30;
  
  cmd = 0;

  if (threshold_min<0)
  {lineTension_offset = lineTension;}
  lineTension = lineTension -lineTension_offset;

 if (fabs(manucmd)>10)
 {
  cmd = manucmd/3;
   if ((lineTension<100)&&(cmd>0))
   {
    cmd= (cmd-20)/3;
   }
 }
 else
 {
  if (gain>0)
  {
    if (lineTension>threshold_max)
    { cmd = (lineTension-threshold_max)*gain/255.;}
    if (lineTension<threshold_min)
    { cmd = (lineTension-threshold_min)*gain/255.*5;}
  }
  else
  {
    cmd = 0;
  }

 }
 byte cmd_max = 40;
 int cmd_min = -70;

 cmd = constrain(cmd, cmd_min, cmd_max);
  sendSabertooth(cmd);

  // Trancannage/winding
  servocmd = 110+30*sin(get_cmd_integral()/127.0*0.1);
  sendServo(servocmd);
  
  
  
 // Debug output
 Serial.print("thmin:");
  Serial.print(threshold_min);
  Serial.print('\t');
  Serial.print("thmax:");
  Serial.print(threshold_max);
  Serial.print('\t');
  Serial.print("gain:");
  Serial.print(gain);
  Serial.print('\t');
  Serial.print("manu:");
  Serial.print(manucmd);
  Serial.print('\t');
  Serial.print("tension:");
  Serial.print(lineTension);
  Serial.print('\t');
  Serial.print("cmd:");
  Serial.println(cmd);
  Serial.print('\t');
  Serial.print("servocmd:");
  Serial.print(servocmd);

}
