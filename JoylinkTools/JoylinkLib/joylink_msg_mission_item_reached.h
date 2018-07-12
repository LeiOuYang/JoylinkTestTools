#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_MISSION_ITEM_REACHED 46

typedef struct _joylink_mission_item_reached {
 uint16_t seq; /*< Sequence*/
}joylink_mission_item_reached;

#define JOYLINK_MSG_ID_MISSION_ITEM_REACHED_LEN 2

static inline uint16_t joylink_msg_mission_item_reached_pack(uint8_t dev_id, joylink_message* msg,
                               uint16_t seq)
{
    joylink_mission_item_reached packet;
    packet.seq = seq;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);

    msg->msgid = JOYLINK_MSG_ID_MISSION_ITEM_REACHED;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
}

static inline void joylink_msg_mission_item_reached_send_struct(joylink_channel chan, const joylink_mission_item_reached* mission_item_reached)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_MISSION_ITEM_REACHED, (const char *)mission_item_reached, JOYLINK_MSG_ID_MISSION_ITEM_REACHED_LEN);
}


static inline uint16_t joylink_msg_mission_item_reached_get_seq(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}
}
