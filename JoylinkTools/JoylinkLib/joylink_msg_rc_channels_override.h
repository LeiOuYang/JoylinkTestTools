#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70

typedef struct _joylink_rc_channels_override {
 uint16_t chan1_raw; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan2_raw; /*< RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan3_raw; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan4_raw; /*< RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan5_raw; /*< RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan6_raw; /*< RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan7_raw; /*< RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan8_raw; /*< RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; /*< Component ID*/
}joylink_rc_channels_override;

#define JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN 18

static inline uint16_t joylink_msg_rc_channels_override_pack(uint8_t dev_id, joylink_message* msg,
                               uint8_t target_system, uint8_t target_id, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw)
{
    joylink_rc_channels_override packet;
    packet.chan1_raw = chan1_raw;
    packet.chan2_raw = chan2_raw;
    packet.chan3_raw = chan3_raw;
    packet.chan4_raw = chan4_raw;
    packet.chan5_raw = chan5_raw;
    packet.chan6_raw = chan6_raw;
    packet.chan7_raw = chan7_raw;
    packet.chan8_raw = chan8_raw;
    packet.target_system = target_system;
    packet.target_id = target_id;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);

    msg->msgid = JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
}


static inline void joylink_msg_rc_channels_override_send_struct(joylink_channel chan, const joylink_rc_channels_override* rc_channels_override)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)rc_channels_override, JOYLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
}



static inline uint8_t joylink_msg_rc_channels_override_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  16);
}

static inline uint8_t joylink_msg_rc_channels_override_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  17);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan1_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan2_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  2);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan3_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  4);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan4_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  6);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan5_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  8);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan6_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  10);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan7_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  12);
}

static inline uint16_t joylink_msg_rc_channels_override_get_chan8_raw(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  14);
}
}
