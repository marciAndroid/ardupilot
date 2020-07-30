This branch is used to generate custom code for the ArduCopter 3.6.8 firmware.

Main modifications:
GCS_Mavlink.cpp:
void GCS_MAVLINK_Copter::handleMessage(mavlink_message_t* msg):
(msg->msgid) == MAVLINK_MSG_ID_COMMAND_INT
MAVLINK_MSG_ID_SET_ATTITUDE_TARGET
 ~ line 1205:
 if ((packet.type_mask & ((1<<7)|(1<<6)|(1<<5)|(1<<1))) != 0)
 instead of: 
 if ((packet.type_mask & ((1<<7)|(1<<6))) != 0)