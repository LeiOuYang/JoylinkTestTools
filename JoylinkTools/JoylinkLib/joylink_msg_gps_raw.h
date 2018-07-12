
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_GPS_RAW 24

typedef struct _joylink_gps_raw {
 uint64_t time_usec; /* 系统运行时间  微秒 */
 int32_t lat; /* 椭球  纬度*10的7次方 */
 int32_t lon; /* 椭球  经度*10的7次方 */
 int32_t alt; /* 平均海拔高度， 注意不是椭球高度  *1000*/
 uint16_t eph; /* 水平精度 GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t epv; /* 垂直精度 GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t vel; /* GPS地速 GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
 uint16_t cog; /* Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t fix_type; /* 定位类型 See the JOY_GPS_FIX_TYPE enum.*/
 uint8_t satellites_visible; /* 可见卫星个数. If unknown, set to 255*/
}joylink_gps_raw;

#define JOYLINK_MSG_ID_GPS_RAW_LEN 30

static inline uint16_t joylink_msg_gps_raw_pack(uint8_t dev_id, joylink_message* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
    joylink_gps_raw packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_GPS_RAW_LEN);

    msg->msgid = JOYLINK_MSG_ID_GPS_RAW;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_GPS_RAW_LEN);
}

static inline void joylink_msg_gps_raw_send_struct(joylink_channel chan, const joylink_gps_raw* gps_raw)
{

    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_GPS_RAW, (const char *)gps_raw, JOYLINK_MSG_ID_GPS_RAW_LEN);

}

static inline uint64_t joylink_msg_gps_raw_get_time_usec(const joylink_message* msg)
{
    return JOY_RETURN_uint64_t(msg,  0);
}

static inline uint8_t joylink_msg_gps_raw_get_fix_type(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  28);
}

static inline int32_t joylink_msg_gps_raw_get_lat(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  8);
}

static inline int32_t joylink_msg_gps_raw_get_lon(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  12);
}

static inline int32_t joylink_msg_gps_raw_get_alt(const joylink_message* msg)
{
    return JOY_RETURN_int32_t(msg,  16);
}

static inline uint16_t joylink_msg_gps_raw_get_eph(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  20);
}

static inline uint16_t joylink_msg_gps_raw_get_epv(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  22);
}

static inline uint16_t joylink_msg_gps_raw_get_vel(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  24);
}

static inline uint16_t joylink_msg_gps_raw_get_cog(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  26);
}
 

static inline uint8_t joylink_msg_gps_raw_get_satellites_visible(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  29);
}
}

