
#pragma once

#define JOYLINK_MSG_ID_CAMERA_TRIGGER 112

typedef struct _joylink_camera_trigger {
 uint64_t time_usec; /*< Timestamp for the image frame in microseconds*/
 uint32_t seq; /*< Image frame sequence*/
}joylink_camera_trigger;

#define JOYLINK_MSG_ID_CAMERA_TRIGGER_LEN 12

static inline uint16_t joylink_msg_camera_trigger_pack(uint8_t dev_id, joylink_message* msg,
                               uint64_t time_usec, uint32_t seq)
{

    joylink_camera_trigger packet;
    packet.time_usec = time_usec;
    packet.seq = seq;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_CAMERA_TRIGGER_LEN);

    msg->msgid = JOYLINK_MSG_ID_CAMERA_TRIGGER;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_CAMERA_TRIGGER_LEN);
}


static inline void joylink_msg_camera_trigger_send_struct(joylink_channel chan, const joylink_camera_trigger* camera_trigger)
{

    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_CAMERA_TRIGGER, (const char *)camera_trigger,JOYLINK_MSG_ID_CAMERA_TRIGGER_LEN);

}



static inline uint64_t joylink_msg_camera_trigger_get_time_usec(const joylink_message* msg)
{
    return JOY_RETURN_uint64_t(msg,  0);
}


static inline uint32_t joylink_msg_camera_trigger_get_seq(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  8);
}



