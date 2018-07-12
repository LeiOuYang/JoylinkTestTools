
#pragma once

#define JOYLINK_MSG_ID_CAMERA_FEEDBACK 180

typedef struct _joylink_camera_feedback {
 uint64_t time_usec; /*< Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)*/
 int32_t lat; /*< Latitude in (deg * 1E7)*/
 int32_t lng; /*< Longitude in (deg * 1E7)*/
 float alt_msl; /*< Altitude Absolute (meters AMSL)*/
 float alt_rel; /*< Altitude Relative (meters above HOME location)*/
 float roll; /*< Camera Roll angle (earth frame, degrees, +-180)*/
 float pitch; /*< Camera Pitch angle (earth frame, degrees, +-180)*/
 float yaw; /*< Camera Yaw (earth frame, degrees, 0-360, true)*/
 float foc_len; /*< Focal Length (mm)*/
 uint16_t img_idx; /*< Image index*/
 uint8_t target_system; /*< System ID*/
 uint8_t cam_idx; /*< Camera ID*/
 uint8_t flags; /*< See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask*/
}joylink_camera_feedback;

#define JOYLINK_MSG_ID_CAMERA_FEEDBACK_LEN 45

static inline uint16_t joylink_msg_camera_feedback_pack(uint8_t dev_id, joylink_message* msg,
                               uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags)
{

    joylink_camera_feedback packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt_msl = alt_msl;
    packet.alt_rel = alt_rel;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.foc_len = foc_len;
    packet.img_idx = img_idx;
    packet.target_system = target_system;
    packet.cam_idx = cam_idx;
    packet.flags = flags;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_CAMERA_FEEDBACK_LEN);

    msg->msgid = JOYLINK_MSG_ID_CAMERA_FEEDBACK;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
}

static inline void joylink_msg_camera_feedback_send_struct(joylink_channel chan, const joylink_camera_feedback* camera_feedback)
{
   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)camera_feedback, JOYLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
}


static inline uint64_t joylink_msg_camera_feedback_get_time_usec(const joylink_message* msg)
{
    return JOY_RETURN_uint64_t(msg,  0);
}

static inline uint8_t joylink_msg_camera_feedback_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  42);
}

static inline uint8_t joylink_msg_camera_feedback_get_cam_idx(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  43);
}

static inline uint16_t joylink_msg_camera_feedback_get_img_idx(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  40);
}

static inline int32_t joylink_msg_camera_feedback_get_lat(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  8);
}

static inline int32_t joylink_msg_camera_feedback_get_lng(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  12);
}

static inline float joylink_msg_camera_feedback_get_alt_msl(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}

static inline float joylink_msg_camera_feedback_get_alt_rel(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline float joylink_msg_camera_feedback_get_roll(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}

static inline float joylink_msg_camera_feedback_get_pitch(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  28);
}

static inline float joylink_msg_camera_feedback_get_yaw(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  32);
}

static inline float joylink_msg_camera_feedback_get_foc_len(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  36);
}


static inline uint8_t joylink_msg_camera_feedback_get_flags(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  44);
}


