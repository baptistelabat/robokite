/*
Author: Baptiste LABAT (Nautilabs)
Project: robokite
Date : March 2013
This file ensures the control of a continuous current motor and gearbox through an H-bridge.
A Pulse Width Modulation is used. The algorithm uses varying cycle frequency to allow control at low rotation speed.
Due to minimum quantity of energy to make the motor turn, the motor will not turn smoothly at low speed.
There is no feedback nor stall control on speed, so the output rotation speed may depend on load torque.
The motor is controlled through a parameter alpha which is signed and vary from -1 to -1.
This parameter can be set thanks to a an analog voltage (set through a potentiometer used as joystick for example) or sent over serial connection
The analog voltage is taken by default.
If the analog voltage is not zero, the motor is left free of moving
To start the serial control send "i" in a serial monitor
Then send the value with a dot (ex "0.5")
To stop the serial control send "o".
There is not guarantee that there is a linear relationship between alpha and the rotation speed
*/

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
int initialSensorValue = 0;

double alphaSigned = 0;
String inputString = "";         // A string to hold incoming data
boolean stringComplete = false;  // Whether the string is complete
boolean isSerialControl = false; 
boolean isManualControl = false;
// Read the analog in value
// The range was reduced 
double sensorValueMin = 255;//0
double sensorValueMax = 767;//1023
double deadBand = 0.05; // Use to ensure zero speed. Should not be zero
long initialTime = 0;
boolean isCycleStarting = false;

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
  if (inputString == "i")
  {
    isSerialControl = true;
    initialSensorValue = analogRead(analogInPin);
  }
  else
  {
   if (inputString == "o")
   {
     isSerialControl = false;
   }
   else
   {
     alphaSigned = StrToFloat(inputString);
   }
  }
  Serial.println(inputString);
  inputString="";
}

void loop()
{
  double alpha = 0;
  digitalWrite(HbridgeEnablePin, HIGH);
  
  alpha = computeLogicState();
  if (isCycleStarting)
  {
    if ((millis()-initialTime) > minTime_ms)
    { 
      isCycleStarting = false;
    } 
     Serial.println("isStarting");
  }
  else
  {
    if ((millis() - initialTime) < minTime_ms/alpha*(1-alpha)+minTime_ms)
    {
          digitalWrite(HbridgeEnablePin, LOW);
          Serial.println("isNotStarting");
    }
    else
    {
      initialTime = millis();
      isCycleStarting = true;
    } 
  }
  alpha = computeLogicState();
  delay(1);


//  // print the results to the serial monitor:
//  Serial.print("alphaManual = " );                       
//  Serial.print(alphaSigned);      
//  Serial.print("\t alphaUsed = ");      
//  Serial.println(alphaSigned);   
}

double computeLogicState()
{
  sensorValue = analogRead(analogInPin);
  // If the analog value is changed, it means that the user wants to take control with the joystick
  // so fallback to joystick control
  if (fabs(initialSensorValue-sensorValue) > 10)
  {
    isSerialControl = false;
  }
	
  // Fallback to manual control if the analog voltage is close to zero (probably unplugged)
  isManualControl = (sensorValue < 10);

  if (isManualControl)  
  {
    // Motor is free to move to enable manual control
    digitalWrite(HbridgeEnablePin, LOW);
  }
  
  // If not in Serial control overwrite the value 
  if (false == isSerialControl)
  {
    alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
  }
  // The saturation ensures the motor can be stopped and avoids division by zero
  double alpha = max(fabs(alphaSigned), deadBand);
  alpha = min(alpha, 1);

  if (alphaSigned > fabs(deadBand))
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
      // Brake ?
      digitalWrite(HbridgeLogicPin1, LOW);
      digitalWrite(HbridgeLogicPin2, LOW);
    }
	// \todo: add an input to enable manual rotation of the motor
  }
  return alpha;
}




