
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_CURRENT 42

typedef struct _joylink_mission_current {
 uint16_t seq; /*< Sequence*/
}joylink_mission_current;

#define JOYLINK_MSG_ID_MISSION_CURRENT_LEN 2

static inline uint16_t joylink_msg_mission_current_pack(uint8_t dev_id, joylink_message* msg, uint16_t seq)
{
    joylink_mission_current packet;
    packet.seq = seq;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_CURRENT_LEN);
		
	  msg->msgid = JOYLINK_MSG_ID_MISSION_CURRENT;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_CURRENT_LEN);
}

static inline void joylink_msg_mission_current_send_struct(joylink_channel chan, const joylink_mission_current* mission_current)
{

   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_CURRENT, (const char *)mission_current,JOYLINK_MSG_ID_MISSION_CURRENT_LEN);
}

static inline uint16_t joylink_msg_mission_current_get_seq(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}
}

