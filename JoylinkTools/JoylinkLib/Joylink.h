/*********************************************************

	Joytonlink Drone data transmission protocol
	
	Note: Copyright belongs to the company-Joyton

*********************************************************/
#pragma once

#ifndef JOYTONLINK_H
#define JOYTONLINK_H


extern "C"
{
#include "LoopQueue.h"
	#define JOYTONLINK_STX 0x24  //'$'
	#define JOYTONLINK_DATA_MAX_LEN 255
	#define JOYTONLINK_CHECK_LEN 1  
	
	/* Data Frame Structure */
	typedef struct _joylink_message
	{
		uint8_t stx;
		uint8_t seq;
		uint8_t devid;
		uint8_t msgid;
		uint8_t len;
		uint8_t crc;
		uint16_t package[256/2];
    }joylink_message;

    extern uint8_t joylink_buff[1024*3];
    extern LoopQueue sendJoylinkDataQueue;

#include "joylink_defines.h"
#include "joylink_common.h"
#include "joylink_msg_heartbeat.h"
#include "joylink_msg_gps_position.h"
#include "joylink_msg_gps_raw.h"
#include "joylink_msg_power_status.h"
#include "joylink_msg_fly_status.h"
#include "joylink_msg_mission_current.h"
#include "joylink_msg_system_time.h"
#include "joylink_msg_command.h"
#include "joylink_msg_command_ack.h"
#include "joylink_msg_mission_set_current.h"
#include "joylink_msg_mission_clear_all.h"
#include "joylink_msg_mission_count.h"
#include "joylink_msg_mission_element.h"
#include "joylink_msg_mission_request.h"
#include "joylink_msg_mission_request_list.h"
#include "joylink_msg_mission_ack.h"
#include "joylink_msg_pressure.h"
#include "joylink_msg_attitude.h"
#include "joylink_msg_request_send_data.h"
#include "joylink_msg_set_mode.h"
#include "joylink_msg_home_position.h"
#include "joylink_msg_set_positon_target_local_ned.h"
#include "joylink_msg_set_position_target_global_int.h"
#include "joylink_msg_landing_target.h"
#include "joylink_msg_set_home_position.h"
#include "joylink_msg_rc_channels_override.h"
#include "joylink_msg_nav_controller_output.h"
#include "joylink_msg_rangefinder.h"
#include "joylink_msg_mission_item_reached.h"
#include "joylink_msg_set_attitude_target.h"
#include "joylink_msg_camera_trigger.h"
#include "joylink_msg_camera_feedback.h"
}
	
#endif

