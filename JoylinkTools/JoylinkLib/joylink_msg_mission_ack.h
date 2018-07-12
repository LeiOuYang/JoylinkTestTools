#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_ACK 47

typedef struct _joylink_mission_ack {
 uint8_t target_system; 
 uint8_t target_id; 
 uint8_t type; /*< See JOY_MISSION_RESULT enum*/
} joylink_mission_ack;

#define JOYLINK_MSG_ID_MISSION_ACK_LEN 3

static inline uint16_t joylink_msg_mission_ack_pack(uint8_t dev_id, joylink_message* msg, uint8_t target_system, uint8_t target_id, uint8_t type)
{
    joylink_mission_ack packet;
    packet.target_system = target_system;
    packet.target_id = target_id;
    packet.type = type;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_ACK_LEN);
    msg->msgid = JOYLINK_MSG_ID_MISSION_ACK;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_ACK_LEN);
}

static inline void joylink_msg_mission_ack_send_struct(joylink_channel chan, const joylink_mission_ack* mission_ack)
{

   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_ACK, (const char *)mission_ack, JOYLINK_MSG_ID_MISSION_ACK_LEN);

}

static inline uint8_t joylink_msg_mission_ack_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  0);
}

static inline uint8_t joylink_msg_mission_ack_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  1);
}

static inline uint8_t joylink_msg_mission_ack_get_type(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  2);
}
}
