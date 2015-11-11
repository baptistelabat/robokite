#include "Ultrasonic.h"
Ultrasonic ultrasonic1(8,9);
Ultrasonic ultrasonic2(10,11);
Ultrasonic ultrasonic3(12,13);
const int alimPin1 =  6;      // the number of the LED pin
const int alimPin2 =  7;      // the number of the LED pin

double d1,d2,d3;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(alimPin1, OUTPUT);
  pinMode(alimPin2, OUTPUT);
  digitalWrite(alimPin1, HIGH);
  digitalWrite(alimPin2, HIGH);
}

void loop()
{
  d1 = ultrasonic1.Ranging(CM);
  d2 = ultrasonic2.Ranging(CM);
  d3 = ultrasonic3.Ranging(CM);
  /*Serial.print("distance(cm) = ");
  Serial.print(d1);
  Serial.print(", ");
  Serial.print(d2);
  Serial.print(", ");*/
  Serial.println(d1);  
  //Serial.println((long)d1);  
    
  delay(10);
}




