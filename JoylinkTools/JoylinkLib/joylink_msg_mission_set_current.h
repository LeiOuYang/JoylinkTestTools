#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_SET_CURRENT 41

typedef struct _joylink_mission_set_current {
 uint16_t seq; /*< Sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; 
}joylink_mission_set_current;

#define JOYLINK_MSG_ID_MISSION_SET_CURRENT_LEN 4

static inline uint16_t joylink_msg_mission_set_current_pack(uint8_t dev_id, joylink_message* msg, uint16_t seq, uint8_t target_system, int8_t target_id)
{
    joylink_mission_set_current packet;
    packet.seq = seq;
		packet.target_id = target_id;
		packet.target_system = target_system;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_SET_CURRENT_LEN);

    msg->msgid = JOYLINK_MSG_ID_MISSION_SET_CURRENT;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_SET_CURRENT_LEN);
}

static inline void joylink_msg_mission_set_current_send_struct(joylink_channel chan, const joylink_mission_set_current* mission_set_current)
{

    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_SET_CURRENT, (const char *)mission_set_current, JOYLINK_MSG_ID_MISSION_SET_CURRENT_LEN);

}

static inline uint8_t joylink_msg_mission_set_current_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  2);
}

static inline uint8_t joylink_msg_mission_set_current_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  3);
}

static inline uint16_t joylink_msg_mission_set_current_get_seq(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}
}
