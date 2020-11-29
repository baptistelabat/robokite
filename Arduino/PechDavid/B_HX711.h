/*
 Inspired from code HX711 demo from Nathan Seidle, SparkFun Electronics

 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE
 
 The HX711 does one thing well: read load cells. T

 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/

#include "HX711.h"

#define HX711_DOUT  12 // or DAT
#define HX711_CLK   11
#define HX711_GND   13


#define OFFSET 318500 //10000 // From raw value when no load
#define SCALE  -60 //140.0  // From calibration
HX711 scale;
long raw_value = 0;
double val;    // variable to read the value from the analog pin



void setupHX711() {
  // Ground connection
  scale.begin(HX711_DOUT, HX711_CLK);
  pinMode(HX711_GND, OUTPUT);
//  Serial.print("HX711_DOUT -> D");
//  Serial.println(HX711_DOUT);
//  Serial.print("HX711_CLK -> D");
//  Serial.println(HX711_CLK);
//  Serial.print("HX711_GND -> D");
//  Serial.println(HX711_GND);
//  Serial.println("HX711_VCC -> 3V3");

   // No calibration, read the raw value
  scale.set_offset(OFFSET);
  scale.set_scale(SCALE);
}
void loopHX711() {
  // 4. Acquire reading without blocking
  if (scale.wait_ready_timeout(1000)) {
    long raw_value = scale.read();
     if (raw_value>0)
      {
      val = scale.get_units();
      Serial.print("Reading: ");
      Serial.print(raw_value); //raw value
      Serial.print(" ");
      Serial.print(val, 1); //scaled and offset after calibration
      Serial.print(" g"); 
      Serial.println();
      }
//      else
//      {
//        Serial.println("HX711 not valid.");
//      }
  } 
//  else {
//   Serial.println("HX711 not found.");
//  }
  //delay(15);

}

double getLineTension()
{
  return val;
}
