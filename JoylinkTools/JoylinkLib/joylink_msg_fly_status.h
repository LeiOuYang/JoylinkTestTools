
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_FLY_STATUS 74

typedef struct _joylink_fly_status {
 float airspeed; /*< Current airspeed in m/s*/
 float groundspeed; /*< Current ground speed in m/s*/
 float alt; /*< Current altitude (MSL), in meters*/
 float climb; /*< Current climb rate in meters/second*/
 int16_t heading; /*< Current heading in degrees, in compass units (0..360, 0=north)*/
 uint16_t throttle; /*< Current throttle setting in integer percent, 0 to 100*/
} joylink_fly_status;

#define JOYLINK_MSG_ID_FLY_STATUS_LEN 20

static inline uint16_t joylink_msg_fly_status_pack(uint8_t dev_id, joylink_message* msg,
                               float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
    joylink_fly_status packet;
    packet.airspeed = airspeed;
    packet.groundspeed = groundspeed;
    packet.alt = alt;
    packet.climb = climb;
    packet.heading = heading;
    packet.throttle = throttle;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_FLY_STATUS_LEN);

    msg->msgid = JOYLINK_MSG_ID_FLY_STATUS;
    return joylink_finalize_message(msg,  dev_id, JOYLINK_MSG_ID_FLY_STATUS_LEN);
}

static inline void joylink_msg_fly_status_send_struct(joylink_channel chan, const joylink_fly_status* fly_status)
{
    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_FLY_STATUS, (const char *)fly_status, JOYLINK_MSG_ID_FLY_STATUS_LEN);

}

static inline float joylink_msg_fly_status_get_airspeed(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  0);
}

static inline float joylink_msg_fly_status_get_groundspeed(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline int16_t joylink_msg_fly_status_get_heading(const joylink_message* msg)
{
    return JOY_RETURN_int16_t(msg,  16);
}

static inline uint16_t joylink_msg_fly_status_get_throttle(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  18);
}

static inline float joylink_msg_fly_status_get_alt(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline float joylink_msg_fly_status_get_climb(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

}
