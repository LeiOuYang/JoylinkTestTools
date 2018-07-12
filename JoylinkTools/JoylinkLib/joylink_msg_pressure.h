#pragma once
extern "C"
{
#define JOYLINK_MSG_ID_SCALED_PRESSURE 29

typedef struct _joylink_pressure {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float press_abs; /*< Absolute pressure (hectopascal)*/
 float press_diff; /*< Differential pressure 1 (hectopascal)*/
 int16_t temperature; /*< Temperature measurement (0.01 degrees celsius)*/
}joylink_pressure;

#define JOYLINK_MSG_ID_PRESSURE_LEN 14

static inline uint16_t joylink_msg_pressure_pack(uint8_t dev_id, joylink_message* msg, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
    joylink_pressure packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_PRESSURE_LEN);

   	msg->msgid = JOYLINK_MSG_ID_SCALED_PRESSURE;
    return joylink_finalize_message(msg, dev_id, JOYLINK_MSG_ID_PRESSURE_LEN);
}


static inline void joylink_msg_pressure_send_struct(joylink_channel chan, const joylink_pressure* scaled_pressure)
{

  joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_SCALED_PRESSURE, (const char *)scaled_pressure, JOYLINK_MSG_ID_PRESSURE_LEN);

}

static inline uint32_t joylink_msg_pressure_get_time_boot_ms(const joylink_message* msg)
{
    return JOY_RETURN_uint32_t(msg,  0);
}


static inline float joylink_msg_pressure_get_press_abs(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  4);
}

static inline float joylink_msg_pressure_get_press_diff(const joylink_message* msg)
{
    return JOY_RETURN_float(msg,  8);
}

static inline int16_t joylink_msg_pressure_get_temperature(const joylink_message* msg)
{
    return JOY_RETURN_int16_t(msg,  12);
}
}
