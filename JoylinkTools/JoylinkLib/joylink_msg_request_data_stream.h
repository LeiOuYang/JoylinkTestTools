#pragma once
extern "C"
{

#define JOYLINK_MSG_ID_REQUEST_DATA_STREAM 66

typedef struct _joylink_request_data_stream {
 uint16_t req_message_rate; /*< The requested message rate*/
 uint8_t target_system; /*< The target requested to send the message stream.*/
 uint8_t target_id; /*< The target requested to send the message stream.*/
 uint8_t req_stream_id; /*< The ID of the requested data stream*/
 uint8_t start_stop; /*< 1 to start sending, 0 to stop sending.*/
}joylink_request_data_stream;

#define JOYLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6

static inline uint16_t joylink_msg_request_data_stream_pack(uint8_t dev_id, joylink_message* msg,
                               uint8_t target_system, uint8_t target_id, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    joylink_request_data_stream packet;
    packet.req_message_rate = req_message_rate;
    packet.target_system = target_system;
    packet.target_id = target_id;
    packet.req_stream_id = req_stream_id;
    packet.start_stop = start_stop;

    memcpy(_JOY_PAYLOAD_NON_CONST(msg), &packet, JOYLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);

    msg->msgid = JOYLINK_MSG_ID_REQUEST_DATA_STREAM;
    return joylink_finalize_message(msg, dev_id,JOYLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);
}

static inline void joylink_msg_request_data_stream_send_struct(joylink_channel chan, const joylink_request_data_stream* request_data_stream)
{

	joylink_finalize_message_chan_send(chan, JOYLINK_MSG_ID_REQUEST_DATA_STREAM, (const char *)request_data_stream, JOYLINK_MSG_ID_REQUEST_DATA_STREAM_LEN);

}


static inline uint8_t joylink_msg_request_data_stream_get_target_system(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  2);
}


static inline uint8_t joylink_msg_request_data_stream_get_target_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  3);
}


static inline uint8_t joylink_msg_request_data_stream_get_req_stream_id(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  4);
}


static inline uint16_t joylink_msg_request_data_stream_get_req_message_rate(const joylink_message* msg)
{
    return JOY_RETURN_uint16_t(msg,  0);
}


static inline uint8_t joylink_msg_request_data_stream_get_start_stop(const joylink_message* msg)
{
    return JOY_RETURN_uint8_t(msg,  5);
}
}
