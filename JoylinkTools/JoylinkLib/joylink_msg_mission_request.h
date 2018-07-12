#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_REQUEST 40

typedef struct _joylink_mission_request {
 uint16_t seq; /*< Sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id;
}joylink_mission_request;

#define JOYLINK_MSG_ID_MISSION_REQUEST_LEN 4

static inline uint16_t joylink_msg_mission_request_pack(uint8_t dev_id, joylink_message* msg, uint8_t target_system, uint8_t target_id, uint16_t seq)
{
    joylink_mission_request packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_id = target_id;
    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_REQUEST_LEN);
    msg->msgid = JOYLINK_MSG_ID_MISSION_REQUEST;
    return joylink_finalize_message(msg, dev_id,JOYLINK_MSG_ID_MISSION_REQUEST_LEN);
}

static inline void joylink_msg_mission_request_send_struct(joylink_channel chan, const joylink_mission_request* mission_request)
{
   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_REQUEST, (const char *)mission_request,JOYLINK_MSG_ID_MISSION_REQUEST_LEN);
}

static inline uint8_t joylink_msg_mission_request_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  2);
}

static inline uint8_t joylink_msg_mission_request_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  3);
}

static inline uint16_t joylink_msg_mission_request_get_seq(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}
}
