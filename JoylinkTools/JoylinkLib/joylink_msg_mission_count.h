#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_COUNT 44

typedef struct _joylink_mission_count {
 uint16_t count; /*< Number of mission items in the sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; 
}joylink_mission_count;

#define JOYLINK_MSG_ID_MISSION_COUNT_LEN 4

static inline uint16_t joylink_msg_mission_count_pack(uint8_t dev_id, joylink_message* msg, uint8_t target_system, uint8_t target_id, uint16_t count)
{
    joylink_mission_count packet;
    packet.count = count;
    packet.target_id = target_id;
		packet.target_system = target_system;
	
    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_COUNT_LEN);

    msg->msgid = JOYLINK_MSG_ID_MISSION_COUNT;
    return joylink_finalize_message(msg, dev_id,JOYLINK_MSG_ID_MISSION_COUNT_LEN);
}

static inline void joylink_msg_mission_count_send_struct(joylink_channel chan, const joylink_mission_count* mission_count)
{
   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_COUNT, (const char *)mission_count, JOYLINK_MSG_ID_MISSION_COUNT_LEN);
}

static inline uint8_t joylink_msg_mission_count_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  3);
}

static inline uint16_t joylink_msg_mission_count_get_count(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}

static inline uint16_t joylink_msg_mission_count_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  2);
}
}
