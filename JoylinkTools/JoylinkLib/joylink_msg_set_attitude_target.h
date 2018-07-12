
#pragma once
extern "C"
{

#define JOYLINK_MSG_ID_SET_ATTITUDE_TARGET 82


typedef struct _joylink_set_attitude_target {
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot*/
 float q[4]; /*< Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/
 float body_roll_rate; /*< Body roll rate in radians per second*/
 float body_pitch_rate; /*< Body roll rate in radians per second*/
 float body_yaw_rate; /*< Body roll rate in radians per second*/
 float thrust; /*< Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; /*< target ID*/
 uint8_t type_mask; /*< Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude*/
}joylink_set_attitude_target;

#define JOYLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN 39

static inline uint16_t joylink_msg_set_attitude_target_pack(uint8_t dev_id, joylink_message* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_id, uint8_t type_mask, const float *q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{

    joylink_set_attitude_target packet;
    packet.time_boot_ms = time_boot_ms;
    packet.body_roll_rate = body_roll_rate;
    packet.body_pitch_rate = body_pitch_rate;
    packet.body_yaw_rate = body_yaw_rate;
    packet.thrust = thrust;
    packet.target_system = target_system;
    packet.target_id = target_id;
    packet.type_mask = type_mask;
    joy_array_memcpy(packet.q, q, sizeof(float)*4);
    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN);

    msg->msgid = JOYLINK_MSG_ID_SET_ATTITUDE_TARGET;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN);
}

static inline void joylink_msg_set_attitude_target_send_struct(joylink_channel chan, const joylink_set_attitude_target* set_attitude_target)
{
    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_SET_ATTITUDE_TARGET, (const char *)set_attitude_target, JOYLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN);
}

static inline uint32_t joylink_msg_set_attitude_target_get_time_boot_ms(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  0);
}

static inline uint8_t joylink_msg_set_attitude_target_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  36);
}

static inline uint8_t joylink_msg_set_attitude_target_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  37);
}

static inline uint8_t joylink_msg_set_attitude_target_get_type_mask(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  38);
}

static inline uint16_t joylink_msg_set_attitude_target_get_q(const joylink_message* msg, float *q)
{
    return JOY_RETURN_float_array(msg, q, 4,  4);
}

static inline float joylink_msg_set_attitude_target_get_body_roll_rate(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline float joylink_msg_set_attitude_target_get_body_pitch_rate(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}

static inline float joylink_msg_set_attitude_target_get_body_yaw_rate(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  28);
}

static inline float joylink_msg_set_attitude_target_get_thrust(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  32);
}
}

