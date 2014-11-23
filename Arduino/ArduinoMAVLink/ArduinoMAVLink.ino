// Arduino MAVLink test code.

#include "/home/bat/sketchbook/libraries/mavlink/ardupilotmega/mavlink.h"        // Mavlink interface
long last_time = 0;
void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
        long t = millis();
        if (fabs(t-last_time) >= 1000 )
        {
          digitalWrite(13, HIGH);
    	  // Pack the message
          // static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
          //	uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
    	  mavlink_msg_heartbeat_pack(100, 200, &msg, MAV_TYPE_KITE, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    	
    	  // Copy the message to send buffer 
    	  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    	
    	  // Send the message (.write sends as bytes) 
    	  Serial.write(buf, len);
          //Serial.println("");
          last_time = t;
        }
        delay(100);
        digitalWrite(13, LOW);
}

void SerialEvent() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over serial 
	
        while(Serial.available() > 0) { 
		uint8_t c = Serial.read();
		
		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
			// Handle message
 			switch(msg.msgid) {
			        case MAVLINK_MSG_ID_SET_MODE: {
			        	// set mode
			        }
			        break;
			        case MAVLINK_MSG_ID_COMMAND_ACK:
					// EXECUTE ACTION
				break;
				default:
					//Do nothing
				break;
			} 
		} 
		// And get the next one
	}
}
