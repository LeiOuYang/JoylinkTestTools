#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED 84

typedef struct _joylink_set_position_target_local_ned {
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot*/
 float x; /*< X Position in NED frame in meters*/
 float y; /*< Y Position in NED frame in meters*/
 float z; /*< Z Position in NED frame in meters (note, altitude is negative in NED)*/
 float vx; /*< X velocity in NED frame in meter / s*/
 float vy; /*< Y velocity in NED frame in meter / s*/
 float vz; /*< Z velocity in NED frame in meter / s*/
 float afx; /*< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afy; /*< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afz; /*< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float yaw; /*< yaw setpoint in rad*/
 float yaw_rate; /*< yaw rate setpoint in rad/s*/
 uint16_t type_mask; /*< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; /*< target ID*/
 uint8_t coordinate_frame; /*< Valid options are: JOY_FRAME_LOCAL_NED = 1, JOY_FRAME_LOCAL_OFFSET_NED = 7, JOY_FRAME_BODY_NED = 8, JOY_FRAME_BODY_OFFSET_NED = 9*/
}joylink_set_position_target_local_ned;

#define JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN 53

static inline uint16_t joylink_msg_set_position_target_local_ned_pack(uint8_t dev_id, joylink_message* msg,
                               uint32_t time_boot_ms, uint8_t target_system, uint8_t target_id, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    joylink_set_position_target_local_ned packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.afx = afx;
    packet.afy = afy;
    packet.afz = afz;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = type_mask;
    packet.target_system = target_system;
    packet.target_id = target_id;
    packet.coordinate_frame = coordinate_frame;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);

    msg->msgid = JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);
}

static inline void joylink_msg_set_position_target_local_ned_send_struct(joylink_channel chan, const joylink_set_position_target_local_ned* set_position_target_local_ned)
{

 joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, (const char *)set_position_target_local_ned,JOYLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN);

}

static inline uint32_t joylink_msg_set_position_target_local_ned_get_time_boot_ms(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  0);
}

static inline uint8_t joylink_msg_set_position_target_local_ned_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  50);
}

static inline uint8_t joylink_msg_set_position_target_local_ned_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  51);
}

static inline uint8_t joylink_msg_set_position_target_local_ned_get_coordinate_frame(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  52);
}

static inline uint16_t joylink_msg_set_position_target_local_ned_get_type_mask(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  48);
}

static inline float joylink_msg_set_position_target_local_ned_get_x(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline float joylink_msg_set_position_target_local_ned_get_y(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline float joylink_msg_set_position_target_local_ned_get_z(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_set_position_target_local_ned_get_vx(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}

static inline float joylink_msg_set_position_target_local_ned_get_vy(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline float joylink_msg_set_position_target_local_ned_get_vz(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}

static inline float joylink_msg_set_position_target_local_ned_get_afx(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  28);
}

static inline float joylink_msg_set_position_target_local_ned_get_afy(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  32);
}

static inline float joylink_msg_set_position_target_local_ned_get_afz(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  36);
}

static inline float joylink_msg_set_position_target_local_ned_get_yaw(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  40);
}

static inline float joylink_msg_set_position_target_local_ned_get_yaw_rate(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  44);
}
}
