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
To start the serial control send "i" in a serial monitor. Optionnaly follow by the frequency of the message (fallbacks to zero if no message)
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
double deadBand = 0.2; // Use to ensure zero speed. Should not be zero
long initialTime = 0;
long lastSerialInputTime = 0;
boolean isCycleStarting = false;
double serialFrequency = 0;

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
  if (inputString[0] == 'i')
  {
    isSerialControl = true;
    initialSensorValue = analogRead(analogInPin);
    if (inputString.substring(1)=="")
    {
      serialFrequency = 0;
    }
    else
    { 
      serialFrequency = StrToFloat(inputString.substring(1));
    } 
  }
  else
  {
   if (inputString == "o")
   {
     isSerialControl = false;
   }
   else
   {
     // To reject incorrect value due to 0. badly read
     if ((StrToFloat(inputString)>=-1) & (StrToFloat(inputString)<=1))
     {
       alphaSigned = StrToFloat(inputString);
       lastSerialInputTime = millis();
       Serial.println(alphaSigned);
     }
   }
  }
  Serial.println(inputString);
  inputString="";
}

void loop()
{
  sensorValue = analogRead(analogInPin);
  setMode();
  computeAlphaSigned();
  computeEnableState();
  computeLogicState();
  
  delay(30);


//  // print the results to the serial monitor:
//  Serial.print("alphaManual = " );                       
//  Serial.print(alphaSigned);      
//  Serial.print("\t alphaUsed = ");      
 //Serial.println(alphaSigned);   
}
void setMode()
{  
  // If the analog value is changed, it means that the user wants to take control with the joystick
  // so fallback to joystick control
  if (fabs(initialSensorValue-sensorValue) > 10)
  {
    isSerialControl = false;
  }
	
  // Fallback to manual control if the analog voltage is close to zero (probably unplugged)
  // Place after hardware limit if possible
  // This avoids to go to full speed if one cable in the potentiometer is unplugged
  // Warning: if the midle cable of the potentiometer is unplugged, the signal is free, and the fault can not be detected
  isManualControl = (sensorValue < 5)|(sensorValue >1023-5);
}

void computeAlphaSigned()
{
  if (serialFrequency != 0)
  { // Fallbacks to zero as we received no message in the expected time (twice the time)
    if ((millis()-lastSerialInputTime) > 2*1/serialFrequency*1000)
    {
      alphaSigned = 0;
    }
  }
    // If not in Serial control overwrite the value 
  if (false == isSerialControl)
  {
    // Compute the value corresponding to the deadband
    double sensorDeadBandMax = sensorValueMin + (deadBand/2.0+0.5)*(sensorValueMax - sensorValueMin);
    double sensorDeadBandMin = sensorValueMin + (-deadBand/2.0+0.5)*(sensorValueMax - sensorValueMin);
    if (sensorValue < sensorDeadBandMin) //strictly to avoid alphaSigned equals zero
    {
      alphaSigned = (sensorValue-sensorValueMin)/(sensorDeadBandMax - sensorValueMin)-1;
    } 
    else
    {
      if (sensorValue > sensorDeadBandMax)
      {
        alphaSigned = (sensorValue-sensorDeadBandMax)/(sensorValueMax - sensorDeadBandMax);
      }
      else
      {
        alphaSigned = 0;
      }
    }
  }
  alphaSigned = min(1, max(-1, alphaSigned));
}

void computeEnableState()
{
  if (isManualControl)  
  {
    // Motor is free to move to enable manual control
    digitalWrite(HbridgeEnablePin, LOW);
  }
  else
  {
    if (alphaSigned == 0) //if alpha is exactly zero, brake dynamically
    {
      digitalWrite(HbridgeEnablePin, HIGH);
    }
    else
    {
      if (isCycleStarting)
      {
        if ((millis()-initialTime) > minTime_ms)
        { 
          digitalWrite(HbridgeEnablePin, HIGH);
          isCycleStarting = false;
        } 
      }
      else
      { 
        // note that alpha can't be zero
        double alpha = fabs(alphaSigned);
        if ((millis() - initialTime) < minTime_ms/alpha*(1-alpha)+minTime_ms)
        {
              digitalWrite(HbridgeEnablePin, LOW);
        }
        else
        {
          digitalWrite(HbridgeEnablePin, HIGH); // or low?
          initialTime = millis();
          isCycleStarting = true;
        } 
      }
    }
  }

}

void computeLogicState()
{
  if (alphaSigned > 0)
  {
    // Forward rotation
    digitalWrite(HbridgeLogicPin1, HIGH);
    digitalWrite(HbridgeLogicPin2, LOW);
  }
  else 
  {
    if (alphaSigned < 0)
    {
	  // Backward rotation
      digitalWrite(HbridgeLogicPin1, LOW);
      digitalWrite(HbridgeLogicPin2, HIGH);
    }
    else
    {
      // Brake ?
      digitalWrite(HbridgeLogicPin1, HIGH); // According to a website i don't remember
      digitalWrite(HbridgeLogicPin2, HIGH); // but hard to check with the motor I have
    }
	// \todo: add an input to enable manual rotation of the motor
  }
}




