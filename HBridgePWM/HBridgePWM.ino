/*
Author: Baptiste LABAT (Nautilabs)
Project: robokite
Date : March 2013
This file ensures the control of a continuous current motor and gearbox through an H-bridge.
A Pulse Width Modulation is used. The algorithm uses varying cycle frequency to allow control at low rotation speed.
Due to minimum quantity of energy to make the motor turn, the motor will not turn smoothly at low speed.
There is now feedback nor stall control on speed, so the output rotation speed may depend on load torque.
The motor is controlled through a parameter alpha which is signed and vary from -1 to -1.
This parameter can be set thanks to a an analog voltage (set through a potentiometer for example) or sent over serial connection
There is not guarantee that there is a linear relationship between alpha and the rotation speed
*/
// \todo this code needs refactorization

// Minimal time to have motor starting at maximum voltage. This will depend on the motor used
// Try to reduce to get a smooth motion
double minTime_ms = 30;

// These constants will not change. Used here to 
// give names to set pin numbers:
const int HbridgeEnablePin =  13;
const int HbridgeLogicPin1 = 4;
const int HbridgeLogicPin2 = 5;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // Value read from the potentiometer

double alphaSigned = 0;
String inputString = "";         // A string to hold incoming data
boolean stringComplete = false;  // Whether the string is complete

void setup()
{
  // Set the digital pin as output:
  pinMode(HbridgeEnablePin, OUTPUT);
  pinMode(HbridgeLogicPin1, OUTPUT);
  pinMode(HbridgeLogicPin2, OUTPUT);
  // Initialize serial communications at 9600 bps:
  Serial.begin(9600); 

}
float StrToFloat(String str){
  char carray[str.length() + 1]; // Determine the size of the array
  str.toCharArray(carray, sizeof(carray)); // Put str into an array
  return atof(carray);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte:
    char inChar = (char)Serial.read();
    // Add it to the inputString:
    inputString += inChar;
    // If the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  alphaSigned = StrToFloat(inputString);
  Serial.println(inputString);
  Serial.println(alphaSigned);
  inputString="";
}

void loop()
{
  digitalWrite(HbridgeEnablePin, HIGH);
  // Read the analog in value
  // The range was reduced 
  double sensorValueMin = 255;//0
  double sensorValueMax = 767;//1023
  double deadBand = 0.05; // Use to ensure zero speed. Should not be zero
  sensorValue = analogRead(analogInPin);
  // \todo: add a selection of voltage or serial input
  // alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
  double alpha = max(fabs(alphaSigned), deadBand);
  alpha = min(alpha, 1);
  if (alphaSigned>fabs(deadBand))
  {
    // Forward rotation
    digitalWrite(HbridgeLogicPin1, HIGH);
    digitalWrite(HbridgeLogicPin2, LOW);
  }
  else 
  {
    if (alphaSigned<-fabs(deadBand))
    {
	  // Backward rotation
      digitalWrite(HbridgeLogicPin1, LOW);
      digitalWrite(HbridgeLogicPin2, HIGH);
    }
    else
    {
	  // Brake
      digitalWrite(HbridgeLogicPin1, LOW);
      digitalWrite(HbridgeLogicPin2, LOW);
    }
	// \todo: add an input to enable manual rotation of the motor
  }
  delay(minTime_ms);

  sensorValue = analogRead(analogInPin);
  //alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
  alpha = max(fabs(alphaSigned), deadBand);
  alpha = min(alpha, 1);
    if (alphaSigned>fabs(deadBand))
  {
    digitalWrite(HbridgeLogicPin1, HIGH);
    digitalWrite(HbridgeLogicPin2, LOW);
  }
  else 
  {
    if (alphaSigned<-fabs(deadBand))
    {
      digitalWrite(HbridgeLogicPin1, LOW);
      digitalWrite(HbridgeLogicPin2, HIGH);
    }
    else
    {
      digitalWrite(HbridgeLogicPin1, LOW);
      digitalWrite(HbridgeLogicPin2, LOW);
    }
  }
  double initialTime = millis();
  double elapsedTime_ms = millis() - initialTime;
  while (elapsedTime_ms < minTime_ms/alpha*(1-alpha)-minTime_ms)
  {
    digitalWrite(HbridgeEnablePin, LOW);
    delay(minTime_ms);
    sensorValue = analogRead(analogInPin);
//    Serial.print("sensorValue = " );                       
//    Serial.println(sensorValue);   
//    //alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
//    Serial.print("alphaSigned = " );                       
//    Serial.println(alphaSigned);   
    alpha = max(fabs(alphaSigned), deadBand);
    alpha = min(alpha, 1);
    if (alphaSigned>fabs(deadBand))
    {
      digitalWrite(HbridgeLogicPin1, HIGH);
      digitalWrite(HbridgeLogicPin2, LOW);
    }
    else 
    {
      if (alphaSigned<-fabs(deadBand))
      {
        digitalWrite(HbridgeLogicPin1, LOW);
        digitalWrite(HbridgeLogicPin2, HIGH);
      }
      else
      {
        digitalWrite(HbridgeLogicPin1, LOW);
        digitalWrite(HbridgeLogicPin2, LOW);
      }
    }
//    Serial.print("alpha = " );                       
//    Serial.println(alpha);   
    elapsedTime_ms = millis() - initialTime;
//    Serial.print("elapsedTime_ms = " );                       
//    Serial.println(elapsedTime_ms);    
  }
  digitalWrite(HbridgeEnablePin, LOW);
  delay(fabs(minTime_ms/alpha*(1-alpha)- elapsedTime_ms));

//  // print the results to the serial monitor:
//  Serial.print("alphaManual = " );                       
//  Serial.print(alphaSigned);      
//  Serial.print("\t alphaUsed = ");      
//  Serial.println(alphaSigned);   
}




