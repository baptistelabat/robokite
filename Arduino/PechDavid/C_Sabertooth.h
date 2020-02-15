#include <SabertoothSimplified.h>

#define SABERTOOTH_PIN 10

SabertoothSimplified ST(Serial3); // Use SWSerial as the serial port.

long cmd_integral;

void setupSabertooth()
{
    Serial3.begin(9600);
	//cmd_integral = 0;
}                                      

void sendSabertooth(int cmd)
{
  ST.motor(2, cmd);  // Go forward at full power.
  cmd_integral = cmd_integral + cmd;
}

long get_cmd_integral()
{ return cmd_integral;}
