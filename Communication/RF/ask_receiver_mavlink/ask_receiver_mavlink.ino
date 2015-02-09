// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

#define USE_MAVLINK //Uncomment this line to use mavlink
#ifdef USE_MAVLINK
  #include "MavlinkForArduino.h"        // Mavlink interface
#endif

static int packet_drops = 0;

RH_ASK driver(4800, 11, 12);
// RECEPTEUR : DATA D11

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");
         
    // initialize the digital pin as an output.
    pinMode(led, OUTPUT); 
}

void loop()
{
  communication_receive();
}
/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
static void communication_receive(void)
{
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  mavlink_message_t msg;
  mavlink_status_t status;
 
  if (driver.recv(buf, &buflen))
  {
    for (int i;i<buflen;i++)
    {
      uint8_t c = buf[i];
      // Try to get a new message
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        // Handle message
 
	switch(msg.msgid)
	{
          case MAVLINK_MSG_ID_HEARTBEAT:
          {
	    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(10);                 // wait for a second
            digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
            delay(10);                 // wait for a second
	  }
	    break;
	  default:
	    //Do nothing
	    break;
	}
      }
    }
  }
 
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;
}
