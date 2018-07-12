#include "serialsendthread.h"
#include <QDebug>

uint8_t joylink_buff[1024*3] = {0};
LoopQueue sendJoylinkDataQueue;

uint8_t mavlink_buff[1024*3] = {0};
LoopQueue sendMavlinkDataQueue;

SerialSendThread::SerialSendThread():
    runFlag(1),
    mavJoySelect(1),
    hexCharFlag(1) /*0-hex,1-char*/
{
    initCharLoopQueue(&sendJoylinkDataQueue, joylink_buff, 1024*3, 0);
    initCharLoopQueue(&sendMavlinkDataQueue, mavlink_buff, 1024*3, 0);
}

void SerialSendThread::run()
{
    uint8_t buff[1024*3];
    QString str = "";
    char updatFlag = 0;
    while(1)
    {
        if(this->runFlag==1)
        {
            if(this->serialPort->isOpen())
            {
                if(this->serialPort->bytesToWrite()==0)
                {
                    if(this->getMavJoySelect()==1)
                    {
                        //qDebug()<<"Joylink send";
                        int count = loopQueueLength(&sendJoylinkDataQueue);
                        if(count>0)
                        {
                            for(int i=0; i<count; ++i)
                            {
                                buff[i] = readCharLoopQueue(&sendJoylinkDataQueue);
                                str += QString::number(buff[i],10) + " ";
                            }
                            emit updateData((const char*)buff,count);
                            this->runFlag = 0;
                            //this->serialPort->write((const char*)buff, count);
                            qDebug()<<str;
                            str = "";
                        }
                    }else if(this->getMavJoySelect()==0)
                    {
                        //qDebug()<<"Mavlink send";
                        int count = loopQueueLength(&sendMavlinkDataQueue);
                        if(count>0)
                        {
                            for(int i=0; i<count; ++i)
                            {
                                buff[i] = readCharLoopQueue(&sendMavlinkDataQueue);
                                str += QString::number(buff[i],10)  + " ";
                            }
                            emit updateData((const char*)buff,count);
                            this->runFlag = 0;
                            //this->serialPort->write((const char*)buff, count);
                            qDebug()<<str;
                            str = "";
                        }
                    }
                }
            }
        }

        this->msleep(10);
    }

}

void SerialSendThread::setRunFlag(char flag)
{
    if(this->runFlag!=flag && (flag==1||flag==0))
    {
        this->runFlag = flag;
    }
}

void SerialSendThread::setSerialPort(QSerialPort *serialPort)
{
    if(serialPort!=NULL)
    {
        this->serialPort = serialPort;
    }
}

char SerialSendThread::getRunFlag()
{
    return this->runFlag;
}

char SerialSendThread::getMavJoySelect()
{
    return this->mavJoySelect;
}

void SerialSendThread::setMavJoySelect(char flag)
{
    if(flag>1) return;
    this->mavJoySelect = flag;
}

