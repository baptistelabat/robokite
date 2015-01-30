// Arduino MAVLink test code.

#include "MavlinkForArduino.h"        // Mavlink interface
long last_time = 0;
int packet_drops;
const int pi = 3.1415;

uint8_t system_id = 100;
uint8_t component_id = 200;
void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
        
        // Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
        long t_us = micros();
        if (fabs(t_us-last_time) >= 100000 )
        {
          digitalWrite(13, HIGH);
    	  // Pack the message
          // static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
          //	uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
    	  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_KITE, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    	
    	  // Copy the message to send buffer 
    	  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    	
    	  // Send the message (.write sends as bytes) 
    	  Serial.write(buf, len);
    
          //static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	  //					       uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
          mavlink_msg_attitude_pack(system_id, component_id, &msg, t_us/1000, 20*pi/180.*sin(2*pi/10.*t_us/1000000.), 10*pi/180.*sin(2*pi/10.*t_us/1000000.), 5*pi/180.*sin(2*pi/10.*t_us/1000000.), 0, 0, 0);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          Serial.write(buf, len);
          
          //static inline uint16_t mavlink_msg_local_position_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	  //	uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
          mavlink_msg_local_position_ned_pack(system_id, component_id, &msg, t_us/1000, 50, 20, 10, 0, 0, 0);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          Serial.write(buf, len);
          
          //mavlink_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	  //   uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
          //mavlink_msg_global_position_int_pack(system_id, component_id, &msg, t_ms, 47.4195461*pi/180*1e7, -2.468938*pi/180*1e7, 0, 0, 0, 0, 0, 0);
          
          //mavlink_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
	  //      uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
          mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, t_us, 4, 47.4195461*1e7 + 20/6300000.*180/pi*sin(2*pi/10.*t_us/1000000.)*1e7, -2.468938*1e7+20/6300000.*180/pi*cos(2*pi/10.*t_us/1000000.)*1e7, 0,  3, 10, 0, 0, 5);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          Serial.write(buf, len);
          
          last_time = t_us;
        }
        delay(10);
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
			          //set mode = mavlink_msg_set_mode_get_mode(&msg);
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
  packet_drops += status.packet_rx_drop_count;
}
