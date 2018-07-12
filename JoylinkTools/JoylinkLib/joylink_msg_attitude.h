#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_ATTITUDE 30

typedef struct _joylink_attitude {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
}joylink_attitude;

#define JOYLINK_MSG_ID_ATTITUDE_LEN 28

static inline uint16_t joylink_msg_attitude_pack(uint8_t dev_id, joylink_message* msg, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    joylink_attitude packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_ATTITUDE_LEN);

    msg->msgid = JOYLINK_MSG_ID_ATTITUDE;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_ATTITUDE_LEN);
}

static inline void joylink_msg_attitude_send_struct(joylink_channel chan, const joylink_attitude* attitude)
{
	joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_ATTITUDE, (const char *)attitude, JOYLINK_MSG_ID_ATTITUDE_LEN);
}

static inline uint32_t joylink_msg_attitude_get_time_boot_ms(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  0);
}

static inline float joylink_msg_attitude_get_roll(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline float joylink_msg_attitude_get_pitch(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline float joylink_msg_attitude_get_yaw(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_attitude_get_rollspeed(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}

static inline float joylink_msg_attitude_get_pitchspeed(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline float joylink_msg_attitude_get_yawspeed(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}
}

