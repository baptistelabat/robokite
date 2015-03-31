// ask_transmitter.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to transmit messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) transmitter with an TX-C1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

RH_ASK driver(4800, 11, 5);
//Transmitter : ATAD D12

uint8_t data[4];  // array of unsigned 8-bit type, holding Joystick readings
void setup()
{
    Serial.begin(9600);	  // Debugging only
    if (!driver.init())
         Serial.println("init failed");
    data[0] = 0;//map(584, 0, 1023, 0, 255);
    data[1] = 0;//map(212, 0, 1023, 0, 255);
    data[2] = 0;//map(212, 0, 1023, 0, 255);    
    data[3] = 0;//map(212, 0, 1023, 0, 255);    
    
}

void loop()
{
    data[0] = data[0] + 1 ;//map(584, 0, 1023, 0, 255);
    data[1] = data[0]*2;
    data[2] = data[0]+data[1];
    data[3] = 0;
  
    driver.send(data, sizeof(data));
    driver.waitPacketSent();
    Serial.println("msg sent");
    delay(20);
}
