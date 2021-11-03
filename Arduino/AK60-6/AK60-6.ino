#include <CAN.h>
#include <ServoInput.h> // github.com/dmadison/ServoInput


int tab[8];
int8_t myIndex;
int packetSize;
// limit data to be within bounds
float P_MIN = -12.5;
float P_MAX = 12.5 ;
float V_MIN = -41.87;
float V_MAX = 41.87;
float T_MIN = -9;
float T_MAX = 9;
float Kp_MIN = 0;
float Kp_MAX = 500;
float Kd_MIN = 0;
float Kd_MAX = 5;
float Test_Pos = 0.0;

int dt_ms = 20;

 float order = 0;
 float setpoint = 0;
 float torque_order = 0;
 float torque_setpoint = 0;
 float slew_rate = 20;
 float torque_slew_rate = 30;
 float position = 0;
ServoInputPin<6> servo;
void setup() {
  Serial.begin(9600);
  //while (!Serial);
  delay(3000);

  //Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3)) {
    //Serial.println("Starting CAN failed!");
    while (1);
  }
  setZero(0x01);
  setMotormode(0x01);
  
  //exitMotormode(0x01);

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  float angle = servo.getAngle();  // get angle of servo (0 - 180)
  readCAN();
  bool positionControl = false;
  float torque_extinction_ratio = 1;
  if (positionControl)
  {
    order = mapfloat(angle, 0, 180, P_MIN, P_MAX);
    Serial.print("order:"); Serial.print(order); Serial.print(", ");
    setpoint = setpoint + constrain(order-setpoint, -slew_rate * dt_ms/1000., slew_rate * dt_ms/1000.);
    
    setCommand(0x01, setpoint, 0.0, 25, 5, 0*setpoint);
  }
  else // torque control
  {
    torque_order = mapfloat(angle, 0, 180, T_MIN, T_MAX);
    torque_setpoint = torque_setpoint + constrain(torque_order-torque_setpoint, -torque_slew_rate * dt_ms/1000., torque_slew_rate * dt_ms/1000.);
    
    float kp_extinction_ratio = 0;

    // Track current position, except if going outside range
    setpoint = constrain(position, -0.9*P_MAX, 0.9*P_MAX);
    
    if (abs(position)<0.5*P_MAX)
    {
      torque_extinction_ratio = 1;
      kp_extinction_ratio = 0;
    }
    else
    {
      torque_extinction_ratio = mapfloat(abs(position), 0.5*P_MAX, P_MAX, 1, 0);
      if (abs(position)<0.9*P_MAX)
      {
        kp_extinction_ratio = 0;
      }
      else
      {
        kp_extinction_ratio = 1;
          
      }
    }
    setCommand(0x01, setpoint, 0.0, 25*kp_extinction_ratio, 1, torque_setpoint*torque_extinction_ratio);
  }
  Serial.print("setpoint:"); Serial.print(setpoint); Serial.print(", ");
  Serial.print("torque_setpoint:"); Serial.print(torque_setpoint*torque_extinction_ratio); Serial.print(", ");
  
  
  Serial.println("");
  delay(dt_ms);
  //
}

int setMotormode(int id){

  CAN.beginPacket(id);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFC);
  
  CAN.endPacket();

  return 1;
}

int exitMotormode(int id){
  CAN.beginPacket(id);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFD);
  
  CAN.endPacket();
  return 1;
}

int setZero(int id){
  CAN.beginPacket(id);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFE);
  
  CAN.endPacket();
  return 1;
}

void setCommand(int id, float p_des, float v_des, float kp, float kd, float t_ff)
{


  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff=fminf(fmaxf(T_MIN, t_ff), T_MAX);

//convert float to unsigned ints
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des,V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // send to CAN bus
  
  CAN.beginPacket(id);
  CAN.write(p_int>>8);
  CAN.write(p_int&0xFF);
  CAN.write(v_int>>4);
  CAN.write(((v_int&0xF)<<4)|(kp_int>>8));
  CAN.write(kp_int&0xFF);
  CAN.write(kd_int>>4);
  CAN.write(((kd_int&0xF)<<4)|(t_int>>8));
  CAN.write(t_int&0xff);

  CAN.endPacket();

}
  

int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
  // Converts a float to an unsigned int, given range and number of bits
  float span = x_max - x_min;
  if (x<x_min)
  { 
    x= x_min;
  }
  else if (x>x_max)
  { 
    x = x_max;
    }
  return (int) ((x-x_min)*((float)((1<<bits)-1)/span));
}

void readCAN()
{
  int packetSize = CAN.parsePacket();

  if (packetSize)
  {
    myIndex = 0;
      while (CAN.available()) 
      {
        tab[myIndex]=(CAN.read());
        myIndex++;
      }
      getData(tab);
  }
}

void getData(int tab[8])
{

  int id = tab[0];
  int p_int = tab[1]<<8|tab[2];
  int v_int = (tab[3]<<4)|(tab[4]>>4);
  int i_int = ((tab[4]&0xF)<<8)|tab[5];
  
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  if (id==1)
  {
   position = p;
  
   float speed = v;
   float torque = i;
   Serial.print("position:"); Serial.print(position); Serial.print(", ");
   Serial.print("speed:"); Serial.print(speed); Serial.print(", ");
   Serial.print("intensity:"); Serial.print(torque); Serial.print(", ");
   }
 }

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
