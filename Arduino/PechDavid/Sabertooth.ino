#include <SabertoothSimplified.h>

SabertoothSimplified ST(Serial3); // Use SWSerial as the serial port.

void setupSabertooth()
{
    Serial3.begin(9600);
}                                      

void sendSabertooth(int cmd)
{
  ST.motor(2, cmd);  // Go forward at full power.
}
