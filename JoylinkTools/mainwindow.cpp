#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QMessageBox>
#include <QMetaType>
#include <QStringList>
#include <QTime>
#include "JoylinkLib/joylink.h"
#include "./MavlinkV1/common/mavlink.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->updateSerialComBox();
    this->serialRecThread = new SerialRecThread();
    this->serialSendThread = new SerialSendThread();
    this->serialPort = new QSerialPort();
    this->parseTextEditUpdateEnable = 1;
    this->srcTextEditUpdateEnable = 0;

//    QTime time = QTime::currentTime();
//    qDebug()<<QString::number(time.msec(),10)+"";
//    qRegisterMetaType<QString>("QString");
/*
    qDebug()<<"sizeof(char)="+QString::number(sizeof(char),10) +\
              "sizeof(uint16_t)="+QString::number(sizeof(uint16_t),10) +\
              "sizeof(float)="+QString::number(sizeof(float),10) +\
              "sizeof(double)="+QString::number(sizeof(double),10) +\
              "sizeof(uint32_t)="+QString::number(sizeof(uint32_t),10);
*/
    connect(this->serialRecThread, SIGNAL(updateSrcText(QString)), this, SLOT(appendTextSrcTextEdit(QString)));
    connect(this->serialSendThread, SIGNAL(updateData(const char*, int)), this, SLOT(updateSerialSendData(const char*, int)));
    connect(this->serialRecThread, SIGNAL(updateWidget1(int, QString)), this, SLOT(updateWidget(int, QString)));
    connect(this->serialRecThread, SIGNAL(updateMissionStatus1(QString)), this, SLOT(updateMissionStatus(QString)));
    connect(this->serialRecThread, SIGNAL(updateTextTextEdit1(QString)), this, SLOT(updateTextTextEdit(QString)));
    connect(this->serialRecThread, SIGNAL(updateArmLabel(QString)), this, SLOT(updateArmLabelText(QString)));
    connect(this->serialRecThread, SIGNAL(sendMessage(int,QString)), this, SLOT(updateUiArray(int, QString)));
}

void MainWindow::updateUiArray(int id, QString str)
{
    switch(id)
    {
        case 1:
        {
            ui->simplePlainTextEdit->setReadOnly(false);
            ui->simplePlainTextEdit->setPlainText("");
            ui->simplePlainTextEdit->setPlainText(str);
            ui->simplePlainTextEdit->setReadOnly(true);
            break;
        }
        case 24:
        {
            ui->GpsPlainTextEdit->setReadOnly(false);
            ui->GpsPlainTextEdit->setPlainText("");
            ui->GpsPlainTextEdit->setPlainText(str);
            ui->GpsPlainTextEdit->setReadOnly(true);
            break;
        }
        case 77:
        {
            ui->commandAckLabel->setText("");
            ui->commandAckLabel->setText(str);
            break;
        }

        case 191:
        {
        qDebug()<<"mag cal progress..."+str;
            bool b = false;
            char id = 0;
            id = (char)str.at(0).digitValue();
            uint32_t d = (uint32_t)str.trimmed().toFloat(&b);
            if(!b) return;
            if(id==0)
                ui->compassCalId1ProgressBar->setValue(d-id*100);
            else if(id=1)
                ui->compassCalId2ProgressBar->setValue(d-id*100);
            break;
        }
        case 192:
        {
            char id = 0;
            id = (char)str.at(3).digitValue();

            if(id==0)
            {
                ui->compassId1OffPlainTextEdit->setReadOnly(false);
                ui->compassId1OffPlainTextEdit->setPlainText("");
                ui->compassId1OffPlainTextEdit->setPlainText(str);
                ui->compassId1OffPlainTextEdit->setReadOnly(true);
            }else if(id==1)
            {
                ui->compassId2OffPlainTextEdit->setReadOnly(false);
                ui->compassId2OffPlainTextEdit->setPlainText("");
                ui->compassId2OffPlainTextEdit->setPlainText(str);
                ui->compassId2OffPlainTextEdit->setReadOnly(true);
            }
            break;
        }

        case 193:
        {
            ui->compassHealthyLabel->setText("");
            ui->compassHealthyLabel->setText(str);
            break;
        }

        case 74:
        {
            ui->HUDTextEdit->setReadOnly(false);
            ui->HUDTextEdit->setText("");
            ui->HUDTextEdit->setText(str);
            ui->HUDTextEdit->setReadOnly(true);
            break;
        }



        default: break;
    }

}

void MainWindow::appendTextSrcTextEdit(QString str)
{
    if(str=="") return;
    if(ui->stopSrcButton->text().trimmed()=="start") return;
    static int count = 0;
    ++count;
    if(count>8)
    {
        ui->dataSrcTextEdit->clear();
        count = 0;
    }
    if(this->srcTextEditUpdateEnable)
    {
        ui->dataSrcTextEdit->setText(ui->dataSrcTextEdit->toPlainText()+str);
        ui->dataSrcTextEdit->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
    }

}

MainWindow::~MainWindow()
{
    delete this->serialRecThread;
    delete this->serialSendThread;
    delete this->serialPort;
    delete ui;
}

void MainWindow::updateSerialComBox()
{
    ui->serialComboBox->clear();
    QList<QSerialPortInfo> portList = QSerialPortInfo::availablePorts();
    QSerialPortInfo portInfo;
qDebug()<<portList.size();
    for (int i = 0; i < portList.size(); ++i)
    {
        portInfo = portList.at(i);
        QString portName = portInfo.portName();
        ui->serialComboBox->insertItem(i,portName);
     }
}

bool MainWindow::openSerialPort(QIODevice::OpenMode openMode)
{
    return this->serialPort->open(openMode);
}

bool MainWindow::setSerialPort(const QString &name, qint32 baudRate, QSerialPort::DataBits dataBits, QSerialPort::StopBits stopBits,QSerialPort::Parity parity, QSerialPort::FlowControl flowControl)
{
    if(this->serialPort==NULL) return false;
    if(this->serialPort->isOpen())
    {
        this->serialPort->close();
        if(this->serialPort->isOpen())
            return false;
    }
    this->serialPort->setPortName(name);
    this->serialPort->setDataBits(dataBits);
    this->serialPort->setParity(parity);
    this->serialPort->setFlowControl(flowControl);
    this->serialPort->setBaudRate(baudRate);
    this->serialPort->setStopBits(stopBits);
qDebug()<<"init the serial ok";

    return true;

}

void MainWindow::on_serialConnectButton_clicked()
{
qDebug()<<"打开按钮有点击";
    if( ui->serialConnectButton->text()=="打开")
    {
        QString portName = ui->serialComboBox->currentText();
        QString baudRateString = ui->baudComboBoX->currentText();
//        int size = sizeof(joylink_heartbeat);
//qDebug()<<"heartbeat size: "<<size;
qDebug()<<"port name: "<<portName<<"\tbaudRate: "<<baudRateString.toLong();
        if(portName!="None"&&portName!=NULL&&baudRateString!=""&&baudRateString!=NULL)
        {
            qint32 baudRate = baudRateString.toLong();
            setSerialPort(portName, baudRate, QSerialPort::Data8, QSerialPort::OneStop, QSerialPort::NoParity, QSerialPort::NoFlowControl);
            if(openSerialPort(QIODevice::ReadWrite))
            {
                ui->serialConnectButton->setText("关闭");
                this->serialRecThread->setSerialPort(this->serialPort);
                this->serialRecThread->setRunFlag(RunFlagEnable);
                this->serialSendThread->setSerialPort(this->serialPort);
                this->serialSendThread->setRunFlag(1);
                ui->dataSrcTextEdit->clear();
                if(this->serialRecThread->isRunning()==false)
                    this->serialRecThread->start();
                if(this->serialSendThread->isRunning()==false)
                {
                    this->serialSendThread->start();
                    this->serialSendThread->request_send_data();
                }
qDebug()<<"open the serial";

            }else
            {
                QMessageBox::information(this, tr("提示对话框"),tr("打开串口失败！"),QMessageBox::Ok);
                return;
            }
        }
    }else{
        /*关闭串口*/
        this->serialPort->close();
        this->serialRecThread->setRunFlag(RunFlagDisable);
        this->serialSendThread->setRunFlag(0);
        ui->serialConnectButton->setText("打开");
qDebug()<<"close the serial";
    }
}

void MainWindow::on_hexRadioButton_clicked()
{
    if(ui->hexRadioButton->isChecked()==true)
    {
        if(this->serialRecThread->hexCharFlag!=0)
        {
            this->serialRecThread->hexCharFlag = 0;

qDebug()<<"hex button"<<this->serialRecThread->charToHex(15);
        }
    }
}

void MainWindow::on_charRadioButton_clicked()
{
    if(ui->charRadioButton->isChecked()==true)
    {
        if(this->serialRecThread->hexCharFlag!=1)
        {
            this->serialRecThread->hexCharFlag = 1;
qDebug()<<"char button";
        }
    }
}

void MainWindow::on_clearSrcButton_clicked()
{
    ui->dataSrcTextEdit->setText("");
}
void MainWindow::on_stopSrcButton_clicked()
{
    if(ui->stopSrcButton->text()=="stop")
    {
        ui->stopSrcButton->setText("start");
        this->srcTextEditUpdateEnable = 0;
//qDebug()<<"src button stop...";
    }else if(ui->stopSrcButton->text()=="start")
    {
        ui->stopSrcButton->setText("stop");
        this->srcTextEditUpdateEnable = 1;
//qDebug()<<"src button start...";
    }
}

void MainWindow::on_setModeButton_clicked()
{
    if(this->serialPort->isOpen())
    {
        int index = ui->modeComBox->currentIndex();
        int mode = 0;
        switch(index)
        {
            case 0:
                mode = JOY_STABLILIZE_MODE;
                break;
            case 1:
                mode = JOY_AUTO_MODE;
                break;
            case 2:
                mode = JOY_GUIDED_MODE;
                break;
            case 3:
                mode = JOY_RTL_MODE;
                break;
            case 4:
                 mode = JOY_LAND_MODE;
                break;
            case 5:
                 mode = JOY_POSHOLD_MODE;
                break;
            default: break;
        }
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            qDebug()<<"joylink set mode";
            joylink_set_mode set_mode;
            set_mode.flight_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            joylink_msg_set_mode_send_struct(JOYLINK_COMM_1, &set_mode);
        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            qDebug()<<"mavlink set mode";
            mavlink_set_mode_t set_mode;
            set_mode.custom_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            mavlink_msg_set_mode_send_struct(MAVLINK_COMM_1, &set_mode);
        }
    }
}

void MainWindow::updateSerialSendData(const char * pc, int len)
{
    if(len==0||pc==NULL) return;

    if(this->serialPort->isOpen())
    {
        if(this->serialPort->bytesToWrite()==0)
        {
qDebug()<<"update send serial";
            this->serialPort->write(pc, len);
            this->serialSendThread->setRunFlag(1);
        }
    }
}

void MainWindow::updateWidget(int id, QString str)
{
    switch(id)
    {
        case 0:
            ui->currentModeLabel->setText(str);
            break;
        case 41:
            ui->currentLabel->setText(str);
            break;
        case 44:
            ui->waypointListCombox->clear();
            for(int i=0; i<str.toUShort(); ++i)
            {
                ui->waypointListCombox->addItem(QString::number(i,10));
            }
            break;
        default: break;
    }
}

void MainWindow::updateMissionStatus(QString str)
{
    ui->waypointStatusLabel->setText(str);
}

void MainWindow::updateTextTextEdit(QString str)
{
    ui->textTexEdit->setReadOnly(false);
    ui->textTexEdit->append("WP: " + str);
    ui->textTexEdit->setReadOnly(true);
}

void MainWindow::on_downWaypointButton_clicked()
{
    if(this->serialPort->isOpen())
    {
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            joylink_mission_request_list request;

            request.target_id = 0x01;
            request.target_system = 0x01;

            joylink_msg_mission_request_list_send_struct(JOYLINK_COMM_1, &request);
            ui->waypointStatusLabel->setText("下载");
        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_mission_request_list_t request;

            request.target_component = 0x01;
            request.target_system = 0x01;

            mavlink_msg_mission_request_list_send_struct(MAVLINK_COMM_1, &request);
            ui->waypointStatusLabel->setText("下载");
        }
    }

qDebug()<<"mission request...";
}

void MainWindow::on_upWaypointButton_clicked()
{
    joylink_mission_count mission_count;

qDebug()<<"mission count...";
}

void MainWindow::on_setCurrentWaypointButton_clicked()
{
    if(this->serialPort->isOpen()){
        uint16_t seq = ui->waypointListCombox->currentText().toUShort();
        QString wayString = ui->textTexEdit->toPlainText().trimmed();
        QStringList wpList;
        QString wpString;
        if(wayString!="")
        {
            wpList = wayString.split("WP: ", QString::SkipEmptyParts);
            wpString = wpList.at(seq);
        }
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            if(this->serialRecThread->current_mode==JOY_GUIDED_MODE)
            {
                joylink_mission_element  mission;
                QStringList strList = wpString.trimmed().split(" ", QString::SkipEmptyParts);
                QString temp;

                temp = strList.at(0);
                mission.command = temp.trimmed().toUShort();
                if(mission.command!=16) return;
                qDebug()<<"waypoint command";
                mission.seq = 0;
                temp = strList.at(2);
                mission.param1 = temp.trimmed().toFloat();
                temp = strList.at(3);
                mission.param2 = temp.trimmed().toFloat();
                temp = strList.at(4);
                mission.param3 = temp.trimmed().toFloat();
                temp = strList.at(5);
                mission.param4 = temp.trimmed().toFloat();
                temp = strList.at(6);
                mission.x = temp.trimmed().toFloat();
                temp = strList.at(7);
                mission.y = temp.trimmed().toFloat();
                temp = strList.at(8);
                mission.z = temp.trimmed().toFloat();
                temp = strList.at(9);
                mission.frame = (uint8_t)(temp.trimmed().toUInt());
                mission.current = 2;
                mission.autocontinue = 0;
                mission.target_id = 0x01;
                mission.target_system = 0x01;
qDebug()<<QString::number(mission.command,10)+" "+QString::number(mission.seq,10)+" "+\
          QString::number(mission.param1,'f',8)+" "+QString::number(mission.param2,'f',8)+" "+\
          QString::number(mission.param3,'f',8)+" "+QString::number(mission.param4,'f',8)+" "+\
          QString::number(mission.x,'f',8)+" "+QString::number(mission.y,'f',8)+" "+\
          QString::number(mission.z,'f',8)+" "+QString::number(mission.autocontinue,10)+" "+\
          QString::number(mission.current,10)+" "+QString::number(mission.frame,10);

          joylink_msg_mission_element_send_struct(JOYLINK_COMM_1, &mission);

            }else if(this->serialRecThread->current_mode = JOY_AUTO_MODE)
            {
                //qDebug()<<"auto mode: waypoint"+QString::number(ui->waypointListCombox->currentText().toUShort(),10);
                joylink_mission_set_current set_way;
                set_way.seq = seq;
                set_way.target_id = 1;
                set_way.target_system = 1;

                joylink_msg_mission_set_current_send_struct(JOYLINK_COMM_1, &set_way);
            }

        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            if(this->serialRecThread->current_mode==JOY_GUIDED_MODE)
            {
                mavlink_mission_item_t  mission;
                QStringList strList = wpString.trimmed().split(" ", QString::SkipEmptyParts);
                QString temp;

                temp = strList.at(0);
                mission.command = temp.trimmed().toUShort();
                if(mission.command!=16) return;
                qDebug()<<"waypoint command";
                mission.seq = 0;
                temp = strList.at(2);
                mission.param1 = temp.trimmed().toFloat();
                temp = strList.at(3);
                mission.param2 = temp.trimmed().toFloat();
                temp = strList.at(4);
                mission.param3 = temp.trimmed().toFloat();
                temp = strList.at(5);
                mission.param4 = temp.trimmed().toFloat();
                temp = strList.at(6);
                mission.x = temp.trimmed().toFloat();
                temp = strList.at(7);
                mission.y = temp.trimmed().toFloat();
                temp = strList.at(8);
                mission.z = temp.trimmed().toFloat();
                temp = strList.at(9);
                mission.frame = (uint8_t)(temp.trimmed().toUInt());
                mission.current = 2;
                mission.autocontinue = 0;
                mission.target_component = 0x01;
                mission.target_system = 0x01;
qDebug()<<QString::number(mission.command,10)+" "+QString::number(mission.seq,10)+" "+\
          QString::number(mission.param1,'f',8)+" "+QString::number(mission.param2,'f',8)+" "+\
          QString::number(mission.param3,'f',8)+" "+QString::number(mission.param4,'f',8)+" "+\
          QString::number(mission.x,'f',8)+" "+QString::number(mission.y,'f',8)+" "+\
          QString::number(mission.z,'f',8)+" "+QString::number(mission.autocontinue,10)+" "+\
          QString::number(mission.current,10)+" "+QString::number(mission.frame,10);

            mavlink_msg_mission_item_send_struct(MAVLINK_COMM_1, &mission);

            }else if(this->serialRecThread->current_mode = JOY_AUTO_MODE)
            {
                mavlink_mission_set_current_t set_way;
                set_way.seq = seq;
                set_way.target_component = 1;
                set_way.target_system = 1;

                mavlink_msg_mission_set_current_send_struct(MAVLINK_COMM_1, &set_way);
            }
          }
    }
}

void MainWindow::on_joylinkRadio_clicked()
{
    this->serialSendThread->setMavJoySelect(1);
    this->serialRecThread->setMavJoySelect(1);
qDebug()<<"joylinkRadio clicked";
}

void MainWindow::on_mavlinkRadio_clicked()
{
    this->serialSendThread->setMavJoySelect(0);
    this->serialRecThread->setMavJoySelect(0);
qDebug()<<"mavlinkRadio clicked";
}

void MainWindow::on_ch1HorizontalSlider_valueChanged(int value)
{

    if(this->serialPort->isOpen())
    {
        uint16_t chan1_raw = (uint16_t)value; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan2_raw = (uint16_t)ui->ch2HorizontalSlider->value(); /*< RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan3_raw = 0; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan4_raw = (uint16_t)ui->ch4HorizontalSlider->value(); /*< RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint8_t target_system = 1; /*< System ID*/
        uint8_t target_id = 1; /*< System ID*/
        uint8_t target_component = 1; /*< Component ID*/
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            joylink_rc_channels_override rc;
            rc.chan1_raw = chan1_raw;
            rc.chan2_raw = chan2_raw;
            rc.chan3_raw = chan3_raw;
            rc.chan4_raw = chan4_raw;
            rc.chan5_raw = rc.chan6_raw = rc.chan7_raw = rc.chan8_raw = 0;
            rc.target_id = rc.target_system = 1;

            joylink_msg_rc_channels_override_send_struct(JOYLINK_COMM_1, &rc);

        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_rc_channels_override_t rc;
            rc.chan1_raw = chan1_raw;
            rc.chan2_raw = chan2_raw;
            rc.chan3_raw = chan3_raw;
            rc.chan4_raw = chan4_raw;
            rc.chan5_raw = rc.chan6_raw = rc.chan7_raw = rc.chan8_raw = 0;
            rc.target_component = rc.target_system = 1;

            mavlink_msg_rc_channels_override_send_struct(MAVLINK_COMM_1, &rc);
        }
    }

}

void MainWindow::on_ch1HorizontalSlider_sliderReleased()
{
    ui->ch1HorizontalSlider->setValue(1500);
}

void MainWindow::on_ch2HorizontalSlider_valueChanged(int value)
{
    if(this->serialPort->isOpen())
    {
        uint16_t chan1_raw = (uint16_t)ui->ch1HorizontalSlider->value(); /*< RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan2_raw = (uint16_t)value; /*< RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan3_raw = 0; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan4_raw = (uint16_t)ui->ch4HorizontalSlider->value(); /*< RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint8_t target_id = 1; /*< System ID*/
        uint8_t target_component = 1; /*< Component ID*/
        uint8_t target_system = 1; /*< Component ID*/
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            joylink_rc_channels_override rc;
            rc.chan1_raw = chan1_raw;
            rc.chan2_raw = chan2_raw;
            rc.chan3_raw = chan3_raw;
            rc.chan4_raw = chan4_raw;
            rc.chan5_raw = rc.chan6_raw = rc.chan7_raw = rc.chan8_raw = 0;
            rc.target_id = rc.target_system = 1;

            joylink_msg_rc_channels_override_send_struct(JOYLINK_COMM_1, &rc);

        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_rc_channels_override_t rc;
            rc.chan1_raw = chan1_raw;
            rc.chan2_raw = chan2_raw;
            rc.chan3_raw = chan3_raw;
            rc.chan4_raw = chan4_raw;
            rc.chan5_raw = rc.chan6_raw = rc.chan7_raw = rc.chan8_raw = 0;
            rc.target_component = rc.target_system = 1;

            mavlink_msg_rc_channels_override_send_struct(MAVLINK_COMM_1, &rc);
        }
    }
}

void MainWindow::on_ch4HorizontalSlider_valueChanged(int value)
{
    if(this->serialPort->isOpen())
    {
        uint16_t chan1_raw = (uint16_t)ui->ch1HorizontalSlider->value(); /*< RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan2_raw = (uint16_t)ui->ch2HorizontalSlider->value();/*< RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan3_raw = 0; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint16_t chan4_raw = (uint16_t)value; /*< RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
        uint8_t target_system = 1; /*< System ID*/
        uint8_t target_id = 1; /*< System ID*/
        uint8_t target_component = 1; /*< Component ID*/
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            joylink_rc_channels_override rc;
            rc.chan1_raw = chan1_raw;
            rc.chan2_raw = chan2_raw;
            rc.chan3_raw = chan3_raw;
            rc.chan4_raw = chan4_raw;
            rc.chan5_raw = rc.chan6_raw = rc.chan7_raw = rc.chan8_raw = 0;
            rc.target_id = rc.target_system = 1;

            joylink_msg_rc_channels_override_send_struct(JOYLINK_COMM_1, &rc);

        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_rc_channels_override_t rc;
            rc.chan1_raw = chan1_raw;
            rc.chan2_raw = chan2_raw;
            rc.chan3_raw = chan3_raw;
            rc.chan4_raw = chan4_raw;
            rc.chan5_raw = rc.chan6_raw = rc.chan7_raw = rc.chan8_raw = 0;
            rc.target_component = rc.target_system = 1;

            mavlink_msg_rc_channels_override_send_struct(MAVLINK_COMM_1, &rc);
        }
    }
}

void MainWindow::on_ch3HorizontalSlider_sliderPressed()
{
    ui->ch3HorizontalSlider->setValue(1500);
}

void MainWindow::on_ch4HorizontalSlider_sliderReleased()
{
    ui->ch4HorizontalSlider->setValue(1500);
}

void MainWindow::on_armButton_clicked()
{
    if(this->serialPort->isOpen())
    {
        if(this->serialSendThread->getMavJoySelect()==1)
        {
            joylink_command  command;

            command.command = MAV_CMD_COMPONENT_ARM_DISARM;
            if(this->serialRecThread->planeArm==1)
                command.param1 = 0.0;
            else
                command.param1 = 1.0;
            command.param2 = 0.0; command.param3 = 0.0;
            command.param4 = 0.0; command.param5 = 0.0;
            command.param6 = 0.0; command.param7 = 0.0;
            command.target_id = 0x01;
            command.target_system = 0x01;
            command.confirmation = 0;

            joylink_msg_command_send_struct(JOYLINK_COMM_1, &command);


        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_command_long_t command;

            command.command = MAV_CMD_COMPONENT_ARM_DISARM;
            if(this->serialRecThread->planeArm==1)
                command.param1 = 0.0;
            else
                command.param1 = 1.0;
            command.param2 = 0.0; command.param3 = 0.0;
            command.param4 = 0.0; command.param5 = 0.0;
            command.param6 = 0.0; command.param7 = 0.0;
            command.target_component = 0x01;
            command.target_system = 0x01;
            command.confirmation = 0;

            mavlink_msg_command_long_send_struct(MAVLINK_COMM_1, &command);
        }
    }
}

void MainWindow::updateArmLabelText(QString str)
{
    ui->armLable->setText(str);
}

void MainWindow::on_speedButton_clicked()
{
    bool b = false;
    QString errorStr = "";

    float speed = ui->speedLineEdit->text().trimmed().toFloat(&b);
    if(b)
    {
        qDebug()<<"change speed to: "+QString::number(speed, 'f', 2);
        if(speed<=20&&speed>=3)
        {
            if(this->serialPort->isOpen())
            {
                if(this->serialSendThread->getMavJoySelect()==1)
                {
                    if(this->serialRecThread->current_mode==JOY_GUIDED_MODE)
                    {
                        joylink_command  command;
                        command.command = JOY_CMD_DO_CHANGE_SPEED;
                        command.confirmation = 0;
                        command.param1 = 1.0;
                        command.param2 = speed;
                        command.param3 = -1.0;
                        command.param4 = 0.0;
                        command.param5 = 0.0;
                        command.param6 = 0.0;
                        command.param7 = 0.0;
                        command.target_id = 1;
                        command.target_system = 1;

                        joylink_msg_command_send_struct(JOYLINK_COMM_1, &command);
                    }

                }else if(this->serialSendThread->getMavJoySelect()==0)
                {
                    if(this->serialRecThread->current_mode==JOY_GUIDED_MODE)
                    {
                        mavlink_command_long_t  command;
                        command.command = JOY_CMD_DO_CHANGE_SPEED;
                        command.confirmation = 0;
                        command.param1 = 1.0;
                        command.param2 = speed;
                        command.param3 = -1.0;
                        command.param4 = 0.0;
                        command.param5 = 0.0;
                        command.param6 = 0.0;
                        command.param7 = 0.0;
                        command.target_component = 1;
                        command.target_system = 1;

                        mavlink_msg_command_long_send_struct(MAVLINK_COMM_1, &command);
                    }

                }
            }
            return;
        }else
        {
            errorStr = tr("速度值大于等于3m/s,并且不能大于20m/s!");
        }
    }else
    {
        errorStr = tr("输入值有误!");
    }
    QMessageBox::information(this, tr("提示对话框"),errorStr,QMessageBox::Ok);
    ui->speedLineEdit->setText("0");
    ui->speedLineEdit->setFocus();
}

void MainWindow::on_altButton_clicked()
{
    bool b = false;
    QString errorStr = "";

    float alt = ui->altLineEdit->text().trimmed().toFloat(&b);
    if(this->serialRecThread->location.lat>90 || this->serialRecThread->location.lng>180 || alt>500)
        return;
    if(b)
    {
        qDebug()<<"change alt to: "+QString::number(alt, 'f', 2);
        if(alt<=1000&&alt>=10)
        {
            if(this->serialPort->isOpen())
            {
                if(this->serialSendThread->getMavJoySelect()==1)
                {
                    if(this->serialRecThread->current_mode==JOY_GUIDED_MODE)
                    {
                        joylink_mission_element  mission;
                        mission.command = 16;
                        mission.seq = 0;
                        mission.param1 = 0.0;
                        mission.param2 = 0.0;
                        mission.param3 = 0.0;
                        mission.param4 = 0.0;
                        mission.x = this->serialRecThread->location.lat;
                        mission.y = this->serialRecThread->location.lng;
                        mission.z = alt;
                        mission.frame = 3;
                        mission.current = 2;
                        mission.autocontinue = 0;
                        mission.target_id = 0x01;
                        mission.target_system = 0x01;

                        joylink_msg_mission_element_send_struct(JOYLINK_COMM_1, &mission);

                    }

                }else if(this->serialSendThread->getMavJoySelect()==0)
                {
                    if(this->serialRecThread->current_mode==JOY_GUIDED_MODE)
                    {
                        mavlink_mission_item_t  mission;
                        mission.command = 16;
                        mission.seq = 0;
                        mission.param1 = 0.0;
                        mission.param2 = 0.0;
                        mission.param3 = 0.0;
                        mission.param4 = 0.0;
                        mission.x = this->serialRecThread->location.lat;
                        mission.y = this->serialRecThread->location.lng;
                        mission.z = alt;
                        mission.frame = 3;
                        mission.current = 2;
                        mission.autocontinue = 0;
                        mission.target_component = 0x01;
                        mission.target_system = 0x01;

                        mavlink_msg_mission_item_send_struct(MAVLINK_COMM_1, &mission);
                    }

                }
            }
            return;
        }else
        {
            errorStr = tr("高度大于10米，并且小于1000米!");
        }
    }else
    {
        errorStr = tr("输入值有误!");
    }
    QMessageBox::information(this, tr("提示对话框"),errorStr,QMessageBox::Ok);
    ui->altLineEdit->setText("0");
    ui->altLineEdit->setFocus();
}

void MainWindow::on_ch2HorizontalSlider_sliderReleased()
{
    ui->ch2HorizontalSlider->setValue(1500);
}

void MainWindow::on_RTLModeButton_clicked()
{
    if(this->serialPort->isOpen())
    {
        int mode = JOY_RTL_MODE;
        if(this->serialRecThread->current_mode==JOY_RTL_MODE) return;

        if(this->serialSendThread->getMavJoySelect()==1)
        {
            //qDebug()<<"joylink set RTL mode";
            joylink_set_mode set_mode;
            set_mode.flight_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            joylink_msg_set_mode_send_struct(JOYLINK_COMM_1, &set_mode);
        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_set_mode_t set_mode;
            set_mode.custom_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            mavlink_msg_set_mode_send_struct(MAVLINK_COMM_1, &set_mode);
        }
    }
}

void MainWindow::on_autoModeButton_clicked()
{
    if(this->serialPort->isOpen())
    {
        int mode = JOY_AUTO_MODE;
        if(this->serialRecThread->current_mode==JOY_AUTO_MODE) return;

        if(this->serialSendThread->getMavJoySelect()==1)
        {
            //qDebug()<<"joylink set auto mode";
            joylink_set_mode set_mode;
            set_mode.flight_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            joylink_msg_set_mode_send_struct(JOYLINK_COMM_1, &set_mode);
        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_set_mode_t set_mode;
            set_mode.custom_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            mavlink_msg_set_mode_send_struct(MAVLINK_COMM_1, &set_mode);
        }
    }
}


void MainWindow::on_autoRtlPushButton_clicked()
{
    if(this->serialPort->isOpen())
    {
        int mode = JOY_RTL_MODE;
        if(this->serialRecThread->current_mode==JOY_RTL_MODE) return;

        if(this->serialSendThread->getMavJoySelect()==1)
        {
            //qDebug()<<"joylink set RTL mode";
            joylink_set_mode set_mode;
            set_mode.flight_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            joylink_msg_set_mode_send_struct(JOYLINK_COMM_1, &set_mode);
        }else if(this->serialSendThread->getMavJoySelect()==0)
        {
            mavlink_set_mode_t set_mode;
            set_mode.custom_mode = mode;
            set_mode.target_system = 1;
            set_mode.base_mode = 1;
            mavlink_msg_set_mode_send_struct(MAVLINK_COMM_1, &set_mode);
        }
    }
}

void MainWindow::on_guidedPushButton_clicked()
{
    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink set mode";
        joylink_set_mode set_mode;
        set_mode.flight_mode = JOY_GUIDED_MODE;
        set_mode.target_system = 1;
        set_mode.base_mode = 1;
        joylink_msg_set_mode_send_struct(JOYLINK_COMM_1, &set_mode);
    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"mavlink set mode";
        mavlink_set_mode_t set_mode;
        set_mode.custom_mode = JOY_GUIDED_MODE;
        set_mode.target_system = 1;
        set_mode.base_mode = 1;
        mavlink_msg_set_mode_send_struct(MAVLINK_COMM_1, &set_mode);
    }
}

void MainWindow::on_frontPushButton_clicked()
{
    bool b = false;
    float distance = (ui->flySpeedLineEdit->text().trimmed().toFloat(&b))/100.0;
    if(!b) return;
    qDebug()<<"change speed to: "+QString::number(distance, 'f', 2);

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mavlink_set_position_target_local_ned_t";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"set x pos meter";
        mavlink_set_position_target_local_ned_t pos_target;
        pos_target.target_system = 1;
        pos_target.target_component = 1;
        pos_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
        pos_target.time_boot_ms = 0;
        pos_target.type_mask = 0b1000000000|0b1000000001;
        pos_target.afx = pos_target.afy = pos_target.afz = 0.0;
        pos_target.vx = pos_target.vy = pos_target.vz = 0.0;
        pos_target.x = distance;
        pos_target.y = pos_target.z = 0;
        pos_target.yaw = 0; pos_target.yaw_rate = 0;

        mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_1, &pos_target);
    }
}

void MainWindow::on_behindPushButton_clicked()
{
    bool b = false;
    float distance = (ui->flySpeedLineEdit->text().trimmed().toFloat(&b))/100.0;
    if(!b) return;
    qDebug()<<"change speed to: "+QString::number(distance, 'f', 2);

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mavlink_set_position_target_local_ned_t";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"set -x pos meter";
        mavlink_set_position_target_local_ned_t pos_target;
        pos_target.target_system = 1;
        pos_target.target_component = 1;
        pos_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
        pos_target.time_boot_ms = 0;
        pos_target.type_mask = 0b1000000000|0b1000000001;
        pos_target.afx = pos_target.afy = pos_target.afz = 0.0;
        pos_target.vx = pos_target.vy = pos_target.vz = 0.0;
        pos_target.x = -distance;
        pos_target.y = pos_target.z = 0;
        pos_target.yaw = 0; pos_target.yaw_rate = 0;

        mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_1, &pos_target);
    }
}

void MainWindow::on_LeftPushButton_clicked()
{
    bool b = false;
    float distance = (ui->flySpeedLineEdit->text().trimmed().toFloat(&b))/100.0;
    if(!b) return;
    qDebug()<<"change speed to: "+QString::number(distance, 'f', 2);

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mavlink_set_position_target_local_ned_t";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"set -y pos meter";
        mavlink_set_position_target_local_ned_t pos_target;
        pos_target.target_system = 1;
        pos_target.target_component = 1;
        pos_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
        pos_target.time_boot_ms = 0;
        pos_target.type_mask = 0b1000000000|0b1000000010;
        pos_target.afx = pos_target.afy = pos_target.afz = 0.0;
        pos_target.vx = pos_target.vy = pos_target.vz = 0.0;
        pos_target.x = 0;
        pos_target.y = -distance;
        pos_target.z = 0;
        pos_target.yaw = 0; pos_target.yaw_rate = 0;

        mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_1, &pos_target);
    }
}

void MainWindow::on_rightPushButton_clicked()
{
    bool b = false;
    float distance = (ui->flySpeedLineEdit->text().trimmed().toFloat(&b))/100.0;
    if(!b) return;
    qDebug()<<"change speed to: "+QString::number(distance, 'f', 2);

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mavlink_set_position_target_local_ned_t";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"set y pos meter";
        mavlink_set_position_target_local_ned_t pos_target;
        pos_target.target_system = 1;
        pos_target.target_component = 1;
        pos_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
        pos_target.time_boot_ms = 0;
        pos_target.type_mask = 0b1000000000|0b1000000010;
        pos_target.afx = pos_target.afy = pos_target.afz = 0.0;
        pos_target.vx = pos_target.vy = pos_target.vz = 0.0;
        pos_target.x = 0;
        pos_target.y = distance;
        pos_target.z = 0;
        pos_target.yaw = 0; pos_target.yaw_rate = 0;

        mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_1, &pos_target);
    }
}

void MainWindow::on_UpPushButton_clicked()
{
    bool b = false;
    float distance = (ui->flySpeedLineEdit->text().trimmed().toFloat(&b))/100.0;
    if(!b) return;
    qDebug()<<"change speed to: "+QString::number(distance, 'f', 2);

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mavlink_set_position_target_local_ned_t";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"set -z pos meter";
        mavlink_set_position_target_local_ned_t pos_target;
        pos_target.target_system = 1;
        pos_target.target_component = 1;
        pos_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
        pos_target.time_boot_ms = 0;
        pos_target.type_mask = 0b1000000000|0b1000000100;
        pos_target.afx = pos_target.afy = pos_target.afz = 0.0;
        pos_target.vx = pos_target.vy = pos_target.vz = 0.0;
        pos_target.x = 0;
        pos_target.y = 0;
        pos_target.z = -distance;
        pos_target.yaw = 0; pos_target.yaw_rate = 0;

        mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_1, &pos_target);
    }
}

void MainWindow::on_DownPushButton_clicked()
{
    bool b = false;
    float distance = (ui->flySpeedLineEdit->text().trimmed().toFloat(&b))/100.0;
    if(!b) return;
    qDebug()<<"change speed to: "+QString::number(distance, 'f', 2);

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mavlink_set_position_target_local_ned_t";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"set z pos meter";
        mavlink_set_position_target_local_ned_t pos_target;
        pos_target.target_system = 1;
        pos_target.target_component = 1;
        pos_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;
        pos_target.time_boot_ms = 0;
        pos_target.type_mask = 0b1000000000|0b1000000100;
        pos_target.afx = pos_target.afy = pos_target.afz = 0.0;
        pos_target.vx = pos_target.vy = pos_target.vz = 0.0;
        pos_target.x = 0;
        pos_target.y = 0;
        pos_target.z = distance;
        pos_target.yaw = 0; pos_target.yaw_rate = 0;

        mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_1, &pos_target);
    }
}

void MainWindow::on_pushButton_clicked()
{
    if(this->serialPort->isOpen()) return;

    updateSerialComBox();
}

void MainWindow::on_compassCalButton_clicked()
{
    if(!this->serialPort->isOpen()) return;

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mag cal";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"mag cal";
        mavlink_command_long_t command;

        command.command = MAV_CMD_DO_START_MAG_CAL;
        if(this->serialSendThread->compass_cal_mask==0)
        {
            ui->compassId1OffPlainTextEdit->setPlainText("Compass cal...");
            ui->compassId2OffPlainTextEdit->setPlainText("Compass cal...");
        }else if(this->serialSendThread->compass_cal_mask==1)
        {
            ui->compassId1OffPlainTextEdit->setPlainText("Compass cal...");
        }
        qDebug()<<"send compass cal...";
        command.param1 = this->serialSendThread->compass_cal_mask;
        command.param2 = 1; command.param3 = 1;
        command.param4 = 0.0; command.param5 = 0.0;
        command.param6 = 0.0; command.param7 = 0.0;
        command.target_component = 0x01;
        command.target_system = 0x01;
        command.confirmation = 0;

        mavlink_msg_command_long_send_struct(MAVLINK_COMM_1, &command);
    }
}

void MainWindow::on_compassAllRadioButton_clicked()
{
    this->serialSendThread->compass_cal_mask = 0;
}

void MainWindow::on_compassExtraRadioButton_clicked()
{
    this->serialSendThread->compass_cal_mask = 1;
}

void MainWindow::on_saveCompassCalPushButton_clicked()
{
    if(!this->serialPort->isOpen()) return;

    if(this->serialSendThread->getMavJoySelect()==1)
    {
        qDebug()<<"joylink mag cal OK";

    }else if(this->serialSendThread->getMavJoySelect()==0)
    {
        qDebug()<<"mag cal Accept OK";
        mavlink_command_long_t command;

        command.command = MAV_CMD_DO_ACCEPT_MAG_CAL;

        command.param1 = this->serialSendThread->compass_cal_mask;
        command.param2 = 0; command.param3 = 0;
        command.param4 = 0.0; command.param5 = 0.0;
        command.param6 = 0.0; command.param7 = 0.0;
        command.target_component = 0x01;
        command.target_system = 0x01;
        command.confirmation = 0;

        mavlink_msg_command_long_send_struct(MAVLINK_COMM_1, &command);
    }
}
