#pragma once
#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN 49

typedef struct __mavlink_gps_global_origin_t {
 int32_t latitude; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t longitude; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
}mavlink_gps_global_origin_t;

#define MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN 12

static inline uint16_t mavlink_msg_gps_global_origin_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude)
{
    mavlink_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

    memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);

    msg->msgid = MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
}

static inline void mavlink_msg_gps_global_origin_send_struct(mavlink_channel_t chan, const mavlink_gps_global_origin_t* gps_global_origin)
{

  joylink_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)gps_global_origin, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);

}


static inline int32_t mavlink_msg_gps_global_origin_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

static inline int32_t mavlink_msg_gps_global_origin_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

static inline int32_t mavlink_msg_gps_global_origin_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

