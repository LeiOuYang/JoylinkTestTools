#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_RANGEFINDER 173

typedef struct _joylink_rangefinder {
 float distance; /*< distance in meters*/
 float voltage; /*< raw voltage if available, zero otherwise*/
}joylink_rangefinder;

#define JOYLINK_MSG_ID_RANGEFINDER_LEN 8

static inline uint16_t joylink_msg_rangefinder_pack(uint8_t dev_id, joylink_message* msg,
                               float distance, float voltage)
{
    joylink_rangefinder packet;
    packet.distance = distance;
    packet.voltage = voltage;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_RANGEFINDER_LEN);
    msg->msgid = JOYLINK_MSG_ID_RANGEFINDER;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_RANGEFINDER_LEN);
}

static inline void joylink_msg_rangefinder_send_struct(joylink_channel chan, const joylink_rangefinder* rangefinder)
{
  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_RANGEFINDER, (const char *)rangefinder, JOYLINK_MSG_ID_RANGEFINDER_LEN);
}

static inline float joylink_msg_rangefinder_get_distance(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  0);
}

static inline float joylink_msg_rangefinder_get_voltage(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}
}
