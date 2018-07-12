
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_ELEMENT 39

typedef struct _joylink_mission_element {
 float param1; /*< PARAM1, see MAV_CMD enum*/
 float param2; /*< PARAM2, see MAV_CMD enum*/
 float param3; /*< PARAM3, see MAV_CMD enum*/
 float param4; /*< PARAM4, see MAV_CMD enum*/
 float x; /*< PARAM5 / local: x position, global: latitude*/
 float y; /*< PARAM6 / y position: global: longitude*/
 float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
 uint16_t seq; /*< Sequence*/
 uint16_t command; /*< The scheduled action for the MISSION. see JOY_CMD in common.xml joylink specs*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_id; /*< Component ID*/
 uint8_t frame; /*< The coordinate system of the MISSION. see JOY_FRAME in joylink_types.h*/
 uint8_t current; /*< false:0, true:1*/
 uint8_t autocontinue; /*< autocontinue to next wp*/
}joylink_mission_element;

#define JOYLINK_MSG_ID_MISSION_ELEMENT_LEN 37

static inline uint16_t joylink_msg_mission_element_pack(uint8_t dev_id, joylink_message* msg, uint8_t target_system, uint8_t target_id, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
    joylink_mission_element packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.seq = seq;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_id = target_id;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_ELEMENT_LEN);

    msg->msgid = JOYLINK_MSG_ID_MISSION_ELEMENT;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_ELEMENT_LEN);
}

static inline void joylink_msg_mission_element_send_struct(joylink_channel chan, const joylink_mission_element* mission_element)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_ELEMENT, (const char *)mission_element, JOYLINK_MSG_ID_MISSION_ELEMENT_LEN);

}

static inline uint8_t joylink_msg_mission_element_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  32);
}

static inline uint8_t joylink_msg_mission_element_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  33);
}

static inline uint16_t joylink_msg_mission_element_get_seq(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  28);
}

static inline uint8_t joylink_msg_mission_element_get_frame(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  34);
}

static inline uint16_t joylink_msg_mission_element_get_command(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  30);
}

static inline uint8_t joylink_msg_mission_element_get_current(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  35);
}

static inline uint8_t joylink_msg_mission_element_get_autocontinue(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  36);
}

static inline float joylink_msg_mission_element_get_param1(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  0);
}

static inline float joylink_msg_mission_element_get_param2(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline float joylink_msg_mission_element_get_param3(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline float joylink_msg_mission_element_get_param4(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_mission_element_get_x(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}


static inline float joylink_msg_mission_element_get_y(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}


static inline float joylink_msg_mission_element_get_z(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}

}
