/*
 Inspired from code HX711 demo from Nathan Seidle, SparkFun Electronics

 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE
 
 The HX711 does one thing well: read load cells. T

 Arduino pin (Nano)
 2    -> HX711 CLK
 3    -> HX711 DAT
 3V3  -> HX711 VCC
 GND  -> HX711 GND
 
 5V   -> Radio VCC
 4    -> Radio GND 
 RX   -> Radio TX
 TX   -> Radio RX
 

 
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/

#include "HX711.h"

#define calibration_factor -7050.0 //This value is obtained using the SparkFun_HX711_Calibration sketch

#define DOUT  12
#define CLK  11
#define GND  13

#define OFFSET 8177300
#define SCALE  -146.7

HX711 scale(DOUT, CLK);

long raw_value;

void setup() {
  pinMode(GND, OUTPUT);
  Serial.begin(57600);
  Serial.println("HX711 scale demo");
  
  // No calibration
  scale.set_offset(0);
  scale.set_scale(1); //Read the raw value
  
  // Result of calibration
  //scale.set_offset(8177300);
  //scale.set_scale(-146.7); 
  

  Serial.println("Readings:");
}

void loop() {
  digitalWrite(GND, LOW);
  raw_value = scale.get_units();
  Serial.print("Reading: ");
  Serial.print(raw_value, 1); //raw value
  Serial.print(" ");
  Serial.print((raw_value-OFFSET)/SCALE, 1); //scaled and offset after calibration
  Serial.print(" g"); 
  Serial.println();
}
