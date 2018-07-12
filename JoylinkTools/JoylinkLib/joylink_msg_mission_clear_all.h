#pragma once

extern "C"
{
#define JOYLINK_MSG_ID_MISSION_CLEAR_ALL 45

typedef struct _joylink_mission_clear_all {
	uint8_t target_system; /*< System ID*/
	uint8_t target_id; /*< Target ID*/
}joylink_mission_clear_all;

#define JOYLINK_MSG_ID_MISSION_CLEAR_ALL_LEN 2

static inline uint16_t joylink_msg_mission_clear_all_pack( uint8_t dev_id, joylink_message* msg, uint8_t target_system, uint8_t target_id)
{
    joylink_mission_clear_all packet;
    packet.target_id = target_id;
		packet.target_system = target_system;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);

    msg->msgid = JOYLINK_MSG_ID_MISSION_CLEAR_ALL;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
}

static inline void joylink_msg_mission_clear_all_send_struct(joylink_channel chan, const joylink_mission_clear_all* mission_clear_all)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_CLEAR_ALL, (const char *)mission_clear_all, JOYLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);

}

static inline uint8_t joylink_msg_mission_clear_all_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  1);
}

static inline uint8_t joylink_msg_mission_clear_all_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  0);
}
}
