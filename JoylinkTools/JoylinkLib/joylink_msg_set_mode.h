
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_SET_MODE 11

typedef struct _joylink_set_mode {
 uint32_t flight_mode; /*< The new autopilot-specific mode. This field can be ignored by an autopilot.*/
 uint8_t target_system; /*< The system setting the mode*/
 uint8_t base_mode; /*< The new base mode*/
}joylink_set_mode;

#define JOYLINK_MSG_ID_SET_MODE_LEN 6

static inline uint16_t joylink_msg_set_mode_pack(uint8_t dev_id, joylink_message* msg,
                               uint8_t target_system, uint8_t base_mode, uint32_t flight_mode)
{

    joylink_set_mode packet;
    packet.flight_mode = flight_mode;
    packet.target_system = target_system;
    packet.base_mode = base_mode;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_SET_MODE_LEN);

    msg->msgid = JOYLINK_MSG_ID_SET_MODE;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_SET_MODE_LEN);
}

static inline void joylink_msg_set_mode_send_struct(joylink_channel chan, const joylink_set_mode* set_mode)
{
   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_SET_MODE, (const char *)set_mode, JOYLINK_MSG_ID_SET_MODE_LEN);
}


static inline uint8_t joylink_msg_set_mode_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  4);
}

static inline uint8_t joylink_msg_set_mode_get_base_mode(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  5);
}

static inline uint32_t joylink_msg_set_mode_get_flight_mode(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  0);
}
}


