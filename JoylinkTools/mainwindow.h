#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QList>
#include "SerialRecThread.h"
#include "Serialsendthread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public slots:
    void appendTextSrcTextEdit(QString str);

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QSerialPort* serialPort;
    SerialRecThread* serialRecThread;
    char parseTextEditUpdateEnable;
    char srcTextEditUpdateEnable;
    SerialSendThread* serialSendThread;

public:
    void updateSerialComBox(void);
    bool openSerialPort(QIODevice::OpenMode openMode);
    bool setSerialPort(const QString &name, qint32 baudRate, QSerialPort::DataBits dataBits, QSerialPort::StopBits stopBits, QSerialPort::Parity parity,QSerialPort::FlowControl flowControl);
private slots:
    void on_serialConnectButton_clicked();
    void on_hexRadioButton_clicked();
    void on_charRadioButton_clicked();
    void on_clearSrcButton_clicked();
    void on_stopSrcButton_clicked();
    void on_setModeButton_clicked();
    void updateSerialSendData(const char*, int);
    void updateWidget(int id, QString str);
    void updateMissionStatus(QString str);
    void updateTextTextEdit(QString str);
    void on_downWaypointButton_clicked();
    void on_upWaypointButton_clicked();
    void on_setCurrentWaypointButton_clicked();
    void on_joylinkRadio_clicked();
    void on_mavlinkRadio_clicked();
    void on_ch1HorizontalSlider_valueChanged(int value);
    void on_ch1HorizontalSlider_sliderReleased();
    void on_ch2HorizontalSlider_valueChanged(int value);
    void on_ch4HorizontalSlider_valueChanged(int value);
    void on_ch3HorizontalSlider_sliderPressed();
    void on_ch4HorizontalSlider_sliderReleased();
    void on_armButton_clicked();
    void updateArmLabelText(QString str);
    void on_speedButton_clicked();
    void on_altButton_clicked();
    void on_ch2HorizontalSlider_sliderReleased();
    void on_RTLModeButton_clicked();
    void on_autoModeButton_clicked();
    void on_autoRtlPushButton_clicked();
    void on_guidedPushButton_clicked();
    void on_frontPushButton_clicked();
    void on_behindPushButton_clicked();
    void on_LeftPushButton_clicked();
    void on_rightPushButton_clicked();
    void on_UpPushButton_clicked();
    void on_DownPushButton_clicked();
    void updateUiArray(int, QString);
};

#endif // MAINWINDOW_H
