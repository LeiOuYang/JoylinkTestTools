#ifndef SERIALSENDTHREAD_H
#define SERIALSENDTHREAD_H
#include <QThread>
#include <QSerialPort>
#include <QQueue>
#include "JoylinkLib/Joylink.h"
#include "./MavlinkV1/common/mavlink.h"

extern uint8_t joylink_buff[1024*3];
extern LoopQueue sendJoylinkDataQueue;
extern uint8_t mavlink_buff[1024*3];
extern LoopQueue sendMavlinkDataQueue;

class SerialSendThread : public QThread
{
    Q_OBJECT
public:
    SerialSendThread();
signals:
    void updateData(const char*, int);

public:
    QSerialPort* serialPort;
    char runFlag;
    char hexCharFlag; /*0-hex,1-char*/
    char mavJoySelect; /*0-mav, 1-joy*/
    char compass_cal_mask;
public:
    void run(void);
    void setRunFlag(char flag);
    void setSerialPort( QSerialPort* serialPort);
    char getRunFlag(void);
    char getMavJoySelect(void);
    void setMavJoySelect(char flag);
    void request_send_data();
    //QString charToHex(quint8 c);
};

#endif // SERIALSENDTHREAD_H
