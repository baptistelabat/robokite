// ask_transmitter.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to transmit messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) transmitter with an TX-C1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

#define USE_MAVLINK //Uncomment this line to use mavlink
#ifdef USE_MAVLINK
  #include "MavlinkForArduino.h"        // Mavlink interface
#endif

RH_ASK driver(4800, 11, 12);
//Transmitter : ATAD D12

uint8_t system_id = 100;
uint8_t component_id = 200;
void setup()
{
    Serial.begin(9600);	  // Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    mavlink_message_t msg; 
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;
    
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_KITE, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    
    driver.send(buf, len);
    driver.waitPacketSent();
    delay(200);
}
