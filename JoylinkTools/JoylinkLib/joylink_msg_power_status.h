
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_POWER_STATUS 1

typedef struct _joylink_power_status {
 uint16_t voltage_battery; /* 电压   单位：毫伏mv*/
 int16_t current_battery; /* 电池电流 1 = 10毫安 , -1表示没有测量电流 */
 int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/
}joylink_power_status;

#define JOYLINK_MSG_ID_POWER_STATUS_LEN 5

static inline uint16_t joylink_msg_power_status_pack(uint8_t dev_id, joylink_message* msg,
                                uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining)
{
    joylink_power_status packet;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.battery_remaining = battery_remaining;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_POWER_STATUS_LEN);

    msg->msgid = JOYLINK_MSG_ID_POWER_STATUS;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_POWER_STATUS_LEN);
}

static inline void joylink_msg_power_status_send_struct(joylink_channel chan, const joylink_power_status* power_status)
{
    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_POWER_STATUS, (const char *)power_status, JOYLINK_MSG_ID_POWER_STATUS_LEN);
}

static inline uint16_t joylink_msg_power_status_get_voltage_battery(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}

static inline int16_t joylink_msg_power_status_get_current_battery(const joylink_message* msg)
{
    return JOY_RETURN_int16_t(msg,  2);
}


static inline int8_t joylink_msg_power_status_get_battery_remaining(const joylink_message* msg)
{
    return JOY_RETURN_int8_t(msg,  4);
}
}
