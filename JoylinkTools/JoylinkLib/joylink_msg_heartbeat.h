
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_HEARTBEAT 0


typedef struct _joylink_heartbeat {
 uint8_t flight_mode; 						/* JOY_FLIGHT_MODE ENUM in joylink_defines.h */
 uint8_t type; 										/* JOY_TYPE ENUM */
 uint8_t system_status; 					/* JOY_STATE ENUM */
 uint8_t base_mode;
} joylink_heartbeat;

#define JOYLINK_MSG_ID_HEARTBEAT_LEN 4

static inline uint16_t joylink_msg_heartbeat_pack(uint8_t devid, joylink_message* msg, uint8_t type, uint8_t flight_mode, uint8_t system_status,uint8_t base_mode)
{
    joylink_heartbeat packet;
    packet.flight_mode = flight_mode;
    packet.type = type;
    packet.system_status = system_status;
    packet.base_mode = base_mode;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_HEARTBEAT_LEN);

    msg->msgid = JOYLINK_MSG_ID_HEARTBEAT;
    return joylink_finalize_message(msg, devid, JOYLINK_MSG_ID_HEARTBEAT_LEN);
}

static inline void joylink_msg_heartbeat_send_struct(joylink_channel chan, const joylink_heartbeat* heartbeat)
{
    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_HEARTBEAT, (const char *)heartbeat, JOYLINK_MSG_ID_HEARTBEAT_LEN);
}

static inline uint8_t joylink_msg_heartbeat_get_type(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  1);
}

static inline uint32_t joylink_msg_heartbeat_get_flight_mode(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  0);
}

static inline uint8_t joylink_msg_heartbeat_get_system_status(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  2);
}

static inline uint8_t joylink_msg_heartbeat_get_base_mode(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  3);
}
}
