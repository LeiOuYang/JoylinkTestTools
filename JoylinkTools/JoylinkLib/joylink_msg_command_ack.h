
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_COMMAND_ACK 77

typedef struct _joylink_command_ack {
 uint16_t command; /*< Command ID, as defined by JOY_CMD enum.*/
 uint8_t result; /*< See JOY_RESULT enum*/
} joylink_command_ack;

#define JOYLINK_MSG_ID_COMMAND_ACK_LEN 3

static inline uint16_t joylink_msg_command_ack_pack(uint8_t devid, joylink_message* msg,
                               uint16_t command, uint8_t result)
{
    joylink_command_ack packet;
    packet.command = command;
    packet.result = result;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_COMMAND_ACK_LEN);

    msg->msgid = JOYLINK_MSG_ID_COMMAND_ACK;
    return joylink_finalize_message(msg, devid, JOYLINK_MSG_ID_COMMAND_ACK_LEN);
}

static inline void joylink_msg_command_ack_send_struct(joylink_channel chan, const joylink_command_ack* command_ack)
{
  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_COMMAND_ACK, (const char *)command_ack, JOYLINK_MSG_ID_COMMAND_ACK_LEN);

}

static inline uint16_t joylink_msg_command_ack_get_command(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}

static inline uint8_t joylink_msg_command_ack_get_result(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  2);
}
}
