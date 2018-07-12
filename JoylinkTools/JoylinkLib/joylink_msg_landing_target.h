#pragma once

extern "C"
{
#define JOYLINK_MSG_ID_LANDING_TARGET 149

typedef struct _joylink_landing_target {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float angle_x; /*< X-axis angular offset (in radians) of the target from the center of the image*/
 float angle_y; /*< Y-axis angular offset (in radians) of the target from the center of the image*/
 float distance; /*< Distance to the target from the vehicle in meters*/
 float size_x; /*< Size in radians of target along x-axis*/
 float size_y; /*< Size in radians of target along y-axis*/
 uint8_t target_num; /*< The ID of the target if multiple targets are present*/
 uint8_t frame; /*< MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.*/
}joylink_landing_target;

#define JOYLINK_MSG_ID_LANDING_TARGET_LEN 30

static inline uint16_t joylink_msg_landing_target_pack(uint8_t dev_id, joylink_message* msg,
                               uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y)
{
    joylink_landing_target packet;
    packet.time_usec = time_usec;
    packet.angle_x = angle_x;
    packet.angle_y = angle_y;
    packet.distance = distance;
    packet.size_x = size_x;
    packet.size_y = size_y;
    packet.target_num = target_num;
    packet.frame = frame;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_LANDING_TARGET_LEN);

    msg->msgid = JOYLINK_MSG_ID_LANDING_TARGET;
    return joylink_finalize_message(msg, dev_id,JOYLINK_MSG_ID_LANDING_TARGET_LEN);
}

static inline void joylink_msg_landing_target_send_struct(joylink_channel chan, const joylink_landing_target* landing_target)
{
   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_LANDING_TARGET, (const char *)landing_target, JOYLINK_MSG_ID_LANDING_TARGET_LEN);
}

static inline uint64_t joylink_msg_landing_target_get_time_usec(const joylink_message* msg)
{
    return JOY_RETURN_uint64_t(msg,  0);
}

static inline uint8_t joylink_msg_landing_target_get_target_num(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  28);
}

static inline uint8_t joylink_msg_landing_target_get_frame(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  29);
}

static inline float joylink_msg_landing_target_get_angle_x(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline float joylink_msg_landing_target_get_angle_y(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_landing_target_get_distance(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}

static inline float joylink_msg_landing_target_get_size_x(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline float joylink_msg_landing_target_get_size_y(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}
}
