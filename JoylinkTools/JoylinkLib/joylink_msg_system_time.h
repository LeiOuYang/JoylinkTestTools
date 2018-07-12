
#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_SYSTEM_TIME 2

typedef struct _joylink_system_time {
 uint64_t time_unix_usec; /*< Timestamp of the master clock in microseconds since UNIX epoch.*/
 uint32_t time_boot_ms; /*< Timestamp of the component clock since boot time in milliseconds.*/
}joylink_system_time;

#define JOYLINK_MSG_ID_SYSTEM_TIME_LEN 12

static inline uint16_t joylink_msg_system_time_pack(uint8_t dev_id, joylink_message* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    joylink_system_time packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_SYSTEM_TIME_LEN);

    msg->msgid = JOYLINK_MSG_ID_SYSTEM_TIME;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_SYSTEM_TIME_LEN);
}

static inline void joylink_msg_system_time_send_struct(joylink_channel chan, const joylink_system_time* system_time)
{
    joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_SYSTEM_TIME, (const char *)system_time, JOYLINK_MSG_ID_SYSTEM_TIME_LEN);
}

static inline uint64_t joylink_msg_system_time_get_time_unix_usec(const joylink_message* msg)
{
    return JOY_RETURN_uint64_t(msg,  0);
}

static inline uint32_t joylink_msg_system_time_get_time_boot_ms(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  8);
}
}

