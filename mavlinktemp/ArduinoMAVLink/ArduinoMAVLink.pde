
#include "../mavlink/include/mavlink.h"        // Mavlink interface


void setup() {
	Serial.begin(9600);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
	int system_type = MAV_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Pack the message
	// mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type);	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);

	mavlink_msg_sys_status_pack(2, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);

	mavlink_msg_gps_raw_int_pack(2,220,&msg,microsSinceEpoch(), 3,latthing,-720990890, 10000,1,1,1000,90,6);
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);

	mavlink_msg_attitude_pack(2, 200, &msg, microsSinceEpoch(), 0, .5, 100, 10.0, 10.00, 10.00);
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
		
	
	comm_receive();
}

void comm_receive() { 
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
			        // case MAVLINK_MSG_ID_ACTION:
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


