// Simple Example Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#define RC_CMD_PIN 7
#define RC_THRESHOLD_PIN 9

#define SABERTOOTH_PIN 10

#define HX711_DOUT  12 // or DAT
#define HX711_CLK   11
#define HX711_GND   13

byte value1;
byte value2;
byte value3;

void setup()
{
  Serial.begin(57600);
  setupSabertooth();
  setupRCReceiver();
  setupHX711();
}


void loop()
{
  loopHX711();
  int cmd, threshold, CLcmd, maxcmd;
  double lineTension, CL;
  lineTension = getLineTension()-50;
  loopRCReceiver();
  threshold = value3;
  cmd = value1-127;
  maxcmd = value2-127;
  maxcmd = constrain(maxcmd, 0, 127);
  CL = maxcmd-threshold/255.0*maxcmd*lineTension/1500.0*10;
  CLcmd = constrain(CL, 0, maxcmd);
  cmd = cmd  - CLcmd;
  Serial.print(lineTension);
  Serial.print('\t');
  Serial.print(threshold);
  Serial.print('\t');
  Serial.println(CL);
  sendSabertooth(cmd);
}
