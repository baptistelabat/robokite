// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

#define RF_DATA_PIN 11
RH_ASK driver(4800, RF_DATA_PIN, 12);
// RECEPTEUR : DATA D11

void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
	// Message with a good checksum received, dump it.
	//driver.printBuffer("Got:", buf, buflen);
        for (byte i = 0; i < buflen; i++) // Si il n'est pas corrompu on l'affiche via Serial
	    Serial.println(buf[i]);
        Serial.println("");
    }

}
