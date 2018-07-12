#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_HOME_POSITION 242

typedef struct _joylink_home_position {
 int32_t latitude; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t longitude; /*< Longitude (WGS84, in degrees * 1E7*/
 int32_t altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
 float x; /*< Local X position of this position in the local coordinate frame*/
 float y; /*< Local Y position of this position in the local coordinate frame*/
 float z; /*< Local Z position of this position in the local coordinate frame*/
 float q[4]; /*< World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground*/
 float approach_x; /*< Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 float approach_y; /*< Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
 float approach_z; /*< Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.*/
}joylink_home_position;

#define JOYLINK_MSG_ID_HOME_POSITION_LEN 52

static inline uint16_t joylink_msg_home_position_pack(uint8_t dev_id, joylink_message* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z)
{

    joylink_home_position packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.approach_x = approach_x;
    packet.approach_y = approach_y;
    packet.approach_z = approach_z;
    joy_array_memcpy(packet.q, q, sizeof(float)*4);
    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_HOME_POSITION_LEN);

    msg->msgid = JOYLINK_MSG_ID_HOME_POSITION;
    return joylink_finalize_message(msg, dev_id,JOYLINK_MSG_ID_HOME_POSITION_LEN);
}

static inline void joylink_msg_home_position_send_struct(joylink_channel chan, const joylink_home_position* home_position)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_HOME_POSITION, (const char *)home_position,  JOYLINK_MSG_ID_HOME_POSITION_LEN);

}

static inline int32_t joylink_msg_home_position_get_latitude(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  0);
}

static inline int32_t joylink_msg_home_position_get_longitude(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  4);
}

static inline int32_t joylink_msg_home_position_get_altitude(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  8);
}

static inline float joylink_msg_home_position_get_x(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_home_position_get_y(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}

static inline float joylink_msg_home_position_get_z(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline uint16_t joylink_msg_home_position_get_q(const joylink_message* msg, float *q)
{
    return JOY_RETURN_float_array(msg, q, 4,  24);
}

static inline float joylink_msg_home_position_get_approach_x(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  40);
}

static inline float joylink_msg_home_position_get_approach_y(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  44);
}

static inline float joylink_msg_home_position_get_approach_z(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  48);
}
}
