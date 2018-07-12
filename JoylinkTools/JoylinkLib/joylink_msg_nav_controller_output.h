#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62

typedef struct _joylink_nav_controller_output {
 float nav_roll; /*< Current desired roll in degrees*/
 float nav_pitch; /*< Current desired pitch in degrees*/
 float alt_error; /*< Current altitude error in meters*/
 float aspd_error; /*< Current airspeed error in meters/second*/
 float xtrack_error; /*< Current crosstrack error on x-y plane in meters*/
 int16_t nav_bearing; /*< Current desired heading in degrees*/
 int16_t target_bearing; /*< Bearing to current MISSION/target in degrees*/
 uint16_t wp_dist; /*< Distance to active MISSION in meters*/
}joylink_nav_controller_output;

#define JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN 26

static inline uint16_t joylink_msg_nav_controller_output_pack(uint8_t dev_id, joylink_message* msg,
                               float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
    joylink_nav_controller_output packet;
    packet.nav_roll = nav_roll;
    packet.nav_pitch = nav_pitch;
    packet.alt_error = alt_error;
    packet.aspd_error = aspd_error;
    packet.xtrack_error = xtrack_error;
    packet.nav_bearing = nav_bearing;
    packet.target_bearing = target_bearing;
    packet.wp_dist = wp_dist;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);

    msg->msgid = JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
}

static inline void joylink_msg_nav_controller_output_send_struct(joylink_channel chan, const joylink_nav_controller_output* nav_controller_output)
{
  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)nav_controller_output, JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
}

static inline float joylink_msg_nav_controller_output_get_nav_roll(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  0);
}

static inline float joylink_msg_nav_controller_output_get_nav_pitch(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline int16_t joylink_msg_nav_controller_output_get_nav_bearing(const joylink_message* msg)
{
    return JOY_RETURN_int16_t(msg,  20);
}

static inline int16_t joylink_msg_nav_controller_output_get_target_bearing(const joylink_message* msg)
{
    return JOY_RETURN_int16_t(msg,  22);
}

static inline uint16_t joylink_msg_nav_controller_output_get_wp_dist(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  24);
}

static inline float joylink_msg_nav_controller_output_get_alt_error(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline float joylink_msg_nav_controller_output_get_aspd_error(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_nav_controller_output_get_xtrack_error(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}
}
