#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_REQUEST_LIST 43

typedef struct _joylink_mission_request_list {
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; /*< Target ID*/
}joylink_mission_request_list;

#define JOYLINK_MSG_ID_MISSION_REQUEST_LIST_LEN 2

static inline uint16_t joylink_msg_mission_request_list_pack(uint8_t dev_id, joylink_message* msg, uint8_t target_system, uint8_t target_id)
{
    joylink_mission_request_list packet;
    packet.target_system = target_system;
    packet.target_id = target_id;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);

    msg->msgid = JOYLINK_MSG_ID_MISSION_REQUEST_LIST;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
}

static inline void joylink_msg_mission_request_list_send_struct(joylink_channel chan, const joylink_mission_request_list* mission_request_list)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_REQUEST_LIST, (const char *)mission_request_list, JOYLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);

}

static inline uint8_t joylink_msg_mission_request_list_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  0);
}

static inline uint8_t joylink_msg_mission_request_list_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  1);
}
}
