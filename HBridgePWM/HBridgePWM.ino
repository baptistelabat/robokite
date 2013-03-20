// constants won't change. Used here to 
// set pin numbers:
const int HbridgeEnablePin =  13;
const int HbridgeLogicPin1 = 4;
const int HbridgeLogicPin2 = 5;
int sensorValue = 0;        // value read from the potentiometer
// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

void setup()
{
  // set the digital pin as output:
  pinMode(HbridgeEnablePin, OUTPUT);
  pinMode(HbridgeLogicPin1, OUTPUT);
  pinMode(HbridgeLogicPin2, OUTPUT);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 

}

void loop()
{
  digitalWrite(HbridgeEnablePin, HIGH);
  double minTime_ms = 30;//Minimal time to have motor starting at maximum voltage
  // read the analog in value:
  double sensorValueMin = 255;//0
  double sensorValueMax = 767;//1023
  double deadBand = 0.05; // Should not be zero
  sensorValue = analogRead(analogInPin);
  double alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
  double alpha = max(fabs(alphaSigned), deadBand);
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
  delay(minTime_ms);

  sensorValue = analogRead(analogInPin);
  alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
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
    Serial.print("sensorValue = " );                       
    Serial.println(sensorValue);   
    alphaSigned = 2*((sensorValue-sensorValueMin)/(sensorValueMax - sensorValueMin)-0.5);  
    Serial.print("alphaSigned = " );                       
    Serial.println(alphaSigned);   
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
    Serial.print("alpha = " );                       
    Serial.println(alpha);   
    elapsedTime_ms = millis() - initialTime;
    Serial.print("elapsedTime_ms = " );                       
    Serial.println(elapsedTime_ms);    
  }
  digitalWrite(HbridgeEnablePin, LOW);
  delay(fabs(minTime_ms/alpha*(1-alpha)- elapsedTime_ms));

  // print the results to the serial monitor:
  Serial.print("alphaManual = " );                       
  Serial.print(alphaSigned);      
  Serial.print("\t alphaUsed = ");      
  Serial.println(alphaSigned);   
}




