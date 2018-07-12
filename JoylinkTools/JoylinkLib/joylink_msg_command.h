#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_COMMAND 76

typedef struct _joylink_command {
 float param1; /*< Parameter 1, as defined by JOY_CMD enum.*/
 float param2; /*< Parameter 2, as defined by JOY_CMD enum.*/
 float param3; /*< Parameter 3, as defined by JOY_CMD enum.*/
 float param4; /*< Parameter 4, as defined by JOY_CMD enum.*/
 float param5; /*< Parameter 5, as defined by JOY_CMD enum.*/
 float param6; /*< Parameter 6, as defined by JOY_CMD enum.*/
 float param7; /*< Parameter 7, as defined by JOY_CMD enum.*/
 uint16_t command; /*< Command ID, as defined by JOY_CMD enum.*/
 uint8_t target_system; /*< System which should execute the command*/
 uint8_t target_id; /*< Component which should execute the command, 0 for all components*/
 uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
} joylink_command;

#define JOYLINK_MSG_ID_COMMAND_LEN 33

static inline uint16_t joylink_msg_command_pack(uint8_t dev_id, joylink_message* msg,
                               uint8_t target_system, uint8_t target_id, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    joylink_command packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_id = target_id;
    packet.confirmation = confirmation;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_COMMAND_LEN);

    msg->msgid = JOYLINK_MSG_ID_COMMAND;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_COMMAND_LEN);
}

static inline void joylink_msg_command_send_struct(joylink_channel chan, const joylink_command* command)
{

   joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_COMMAND, (const char *)command, JOYLINK_MSG_ID_COMMAND_LEN);
}

static inline uint8_t joylink_msg_command_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  30);
}

static inline uint8_t joylink_msg_command_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  31);
}

static inline uint16_t joylink_msg_command_get_command(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  28);
}

static inline uint8_t joylink_msg_command_get_confirmation(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  32);
}

static inline float joylink_msg_command_get_param1(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  0);
}

static inline float joylink_msg_command_get_param2(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline float joylink_msg_command_get_param3(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}


static inline float joylink_msg_command_get_param4(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  12);
}

static inline float joylink_msg_command_get_param5(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  16);
}

static inline float joylink_msg_command_get_param6(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  20);
}

static inline float joylink_msg_command_get_param7(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  24);
}
}
