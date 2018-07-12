#ifndef SINGLESERIAL_H
#define SINGLESERIAL_H
#include <QThread>
#include <QIODevice>
#include <QSerialPort>

enum RunFlag{RunFlagDisable,RunFlagEnable};
typedef struct _location{
    float lat;
    float lng;
    int32_t alt;
}Location;

class SerialRecThread : public QThread
{
    Q_OBJECT
signals:
    void updateParseText(QString str);
    void updateSrcText(QString str);
    void updateWidget1(int id, QString str);
    void updateMissionStatus1(QString str);
    void updateTextTextEdit1(QString str);
    void updateArmLabel(QString str);
    void sendMessage(int id, QString msg);

public:
    QSerialPort* serialPort;
    RunFlag runFlag;
    char hexCharFlag; /*0-hex,1-char*/
    uint16_t missionCount;
    char mavJoySelect;
    char planeArm; /* 0-armed, 1-disarmed */
    uint8_t current_mode;
    Location location;
public:
    void run(void);
    void setRunFlag(RunFlag flag);
    void setSerialPort( QSerialPort* serialPort);
    RunFlag getRunFlag(void);
    QString charToHex(quint8 c);
    char getMavJoySelect(void);
    void setMavJoySelect(char flag);
    void setLocation(float lat, float lng, int32_t alt);
public:
    SerialRecThread(){hexCharFlag = 0;missionCount=0;mavJoySelect=1;planeArm=0;current_mode=0;location={99.99,999.999,5000};};
};

#endif // SINGLESERIAL_H
