#include "SerialRecThread.h"
#include <QDebug>
#include "joylinklib/joylink.h"
#include "./MavlinkV1/common/mavlink.h"


void SerialRecThread::run()
{
    joylink_message r_message;
    joylink_status r_joylink_status;
    mavlink_message_t m_message;
    mavlink_status_t m_mavlink_status;
    QString srcString;
    QString parseString;
    QString msgString;
    while(1)
    {
        //qDebug()<<"serial rec thread...";
        this->msleep(1);
        /* 获取串口数据，并且解析，将解析到的数据显示至界面 */
        while(this->runFlag==RunFlagEnable)
        {
            //qDebug()<<"serial rec thread...";
            if(this->serialPort->isOpen())
            {
                 qint64 dataCount = this->serialPort->bytesAvailable(); /*获取有效数据个数*/
    //qDebug()<<dataCount;
                 if(dataCount<=0) continue;

                 QByteArray byteArray = this->serialPort->read(dataCount); /* 获取数据 */
                 parseString = "";
                 srcString = "";
                 msgString = "";

                 /*数据解析处理代码*/
                 for(int i=0; i<byteArray.size(); ++i)
                 {
                    if(this->hexCharFlag==0)
                    {
                        srcString += this->charToHex((quint8)byteArray.at(i))+" ";
    //qDebug()<<srcString;
                    }
                    if(this->hexCharFlag==1)
                    {
                        srcString += byteArray.at(i);
    //qDebug()<<srcString;
                    }

                    if(this->getMavJoySelect()==1)
                    {
                        if(joylink_parse_char(JOYLINK_COMM_1, byteArray.at(i), &r_message, &r_joylink_status)==JOYLINK_FRAMING_OK)
                        {
                            switch(r_message.msgid)
                            {
                                case JOYLINK_MSG_ID_HEARTBEAT: /*#0*/
                                {
                                    uint8_t flight_mode = joylink_msg_heartbeat_get_flight_mode(&r_message);
                                    uint8_t type = joylink_msg_heartbeat_get_type(&r_message);
                                    uint8_t system_status = joylink_msg_heartbeat_get_system_status(&r_message);
                                    uint8_t base_mode = joylink_msg_heartbeat_get_base_mode(&r_message);
                                    //qDebug()<<"HeartBeat..."+QString::number(base_mode, 10);
                                    QString flight_mode_string = "";
                                    QString arm_string = "";
                                    this->current_mode = flight_mode;
                                    switch (flight_mode)
                                    {
                                        case JOY_STABLILIZE_MODE:
                                            flight_mode_string = "STABLIZE";
                                            break;
                                        case JOY_AUTO_MODE:
                                            flight_mode_string = "AUTO";
                                            break;
                                        case JOY_GUIDED_MODE:
                                            flight_mode_string = "GUIDED";
                                            break;
                                        case JOY_RTL_MODE:
                                            flight_mode_string = "RTL";
                                            break;
                                        case JOY_LAND_MODE:
                                            flight_mode_string = "LAND";
                                            break;
                                        case JOY_POSHOLD_MODE:
                                            flight_mode_string = "POSHOLD";
                                            break;

                                        default:
                                            flight_mode_string = "UNKNOW";
                                            break;
                                    }
                                    if(base_mode>>7==1)
                                    {
                                        this->planeArm = 1;
                                        arm_string += tr("已解锁");

                                    }else{

                                        this->planeArm = 0;
                                        arm_string += tr("已加锁");
                                    }

                                    emit updateWidget1(0, flight_mode_string);
                                    emit updateArmLabel(arm_string);
                                    parseString = "HeartBeart>>>\r\n fight_mode: "+flight_mode_string+" type: "+QString::number(type,10)+" status: " + QString::number(system_status,10)+"base_mode:"+QString::number(base_mode,10);
                                    msgString = flight_mode_string +" "+arm_string;
                                    emit sendMessage(0, msgString);
                                    break;
                                }
                                case JOYLINK_MSG_ID_ATTITUDE: /*#30*/
                                {
                                    uint32_t time_boot_ms = joylink_msg_attitude_get_time_boot_ms(&r_message);
                                    float roll = joylink_msg_attitude_get_roll(&r_message);
                                    float pitch = joylink_msg_attitude_get_pitch(&r_message);
                                    float yaw = joylink_msg_attitude_get_yaw(&r_message);
                                    float rollspeed = joylink_msg_attitude_get_rollspeed(&r_message);
                                    float pitchspeed = joylink_msg_attitude_get_pitchspeed(&r_message);
                                    float yawspeed = joylink_msg_attitude_get_yawspeed(&r_message);

                                    parseString += "Attitude>>>\r\ntime: "+QString::number(time_boot_ms,10)+"ms "+\
                                                   "yaw: "+QString::number(yaw)+"rad "+\
                                                   "pitch: "+QString::number(pitch)+"rad "+\
                                                   "roll: "+QString::number(roll)+"rad "+\
                                                   "yawspeed: "+QString::number(yawspeed)+"rad/s "+\
                                                   "pitchspeed: "+QString::number(pitchspeed)+"rad/s "+\
                                                   "rollspeed: "+QString::number(rollspeed)+"rad/s ";
                                    break;
                                }
                                case JOYLINK_MSG_ID_GPS_POSITION: /*#33*/
                                {
                                    uint32_t time_boot_ms = joylink_msg_gps_position_get_time_boot_ms(&r_message);
                                    int32_t lat = joylink_msg_gps_position_get_lat(&r_message);
                                    int32_t lon = joylink_msg_gps_position_get_lon(&r_message);
                                    int32_t alt = joylink_msg_gps_position_get_alt(&r_message);
                                    int32_t relative_alt = joylink_msg_gps_position_get_relative_alt(&r_message);
                                    int16_t vx = joylink_msg_gps_position_get_vx(&r_message);
                                    int16_t vy = joylink_msg_gps_position_get_vy(&r_message);
                                    int16_t vz = joylink_msg_gps_position_get_vz(&r_message);
                                    uint16_t hdg = joylink_msg_gps_position_get_hdg(&r_message);

                                    this->setLocation(lat*1.0/10e7, lon*1.0/10e7, alt/100);

                                    parseString += "GPS POS>>>\r\ntime: "+QString::number(time_boot_ms,10)+"ms "+\
                                                   "lat: "+QString::number(lat,10)+" "+\
                                                   "lon: "+QString::number(lon,10)+" "+\
                                                   "alt: "+QString::number(alt,10)+"cm "+\
                                                   "relative_alt: "+QString::number(relative_alt,10)+"cm "+\
                                                   "vx: "+QString::number(vx,10)+"cm/s "+\
                                                   "vy: "+QString::number(vy,10)+"cm/s "+\
                                                   "vz: "+QString::number(vz,10)+"cm/s "+\
                                                   "hdg: "+QString::number(hdg,10)+"d ";
                                    msgString += QString::number(lat,10)+" "+QString::number(lon,10)+" "+QString::number(alt,10)+" "+\
                                                 QString::number(relative_alt,10);
                                    emit sendMessage(33, msgString);
                                    break;
                                }
                                case JOYLINK_MSG_ID_GPS_RAW: /*#24*/
                                {
                                    uint64_t time_usec = joylink_msg_gps_raw_get_time_usec(&r_message); /* 系统运行时间  微秒 */
                                    int32_t lat = joylink_msg_gps_raw_get_lat(&r_message); /* 椭球  纬度*10的7次方 */
                                    int32_t lon = joylink_msg_gps_raw_get_lon(&r_message); /* 椭球  经度*10的7次方 */
                                    int32_t alt = joylink_msg_gps_raw_get_alt(&r_message); /* 平均海拔高度， 注意不是椭球高度  *1000*/
                                    uint16_t eph = joylink_msg_gps_raw_get_eph(&r_message); /* 水平精度 GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
                                    uint16_t epv = joylink_msg_gps_raw_get_epv(&r_message); /* 垂直精度 GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
                                    uint16_t vel = joylink_msg_gps_raw_get_vel(&r_message); /* GPS地速 GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
                                    uint16_t cog = joylink_msg_gps_raw_get_cog(&r_message); /* Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
                                    uint8_t fix_type = joylink_msg_gps_raw_get_fix_type(&r_message); /* 定位类型 See the JOY_GPS_FIX_TYPE enum.*/
                                    uint8_t satellites_visible = joylink_msg_gps_raw_get_satellites_visible(&r_message); /* 可见卫星个数. If unknown, set to 255*/
                                    QString fix_type_string = "";
                                    if(fix_type==JOY_GPS_FIX_TYPE_NO_GPS)
                                    {
                                        fix_type_string = "NOGPS";
                                    }else if(fix_type==JOY_GPS_FIX_TYPE_NO_FIX)
                                    {
                                        fix_type_string = "NOFIX";
                                    }else if(fix_type==JOY_GPS_FIX_TYPE_2D_FIX)
                                    {
                                        fix_type_string = "2DFIX";
                                    }else if(fix_type==JOY_GPS_FIX_TYPE_3D_FIX)
                                    {
                                        fix_type_string = "3DFIX";
                                    }else if(fix_type==JOY_GPS_FIX_TYPE_DGPS)
                                    {
                                        fix_type_string = "DGPS";
                                    }

                                    parseString += "GPS POS>>>\r\ntime: "+QString::number(time_usec,10)+"us "+\
                                                   "lat: "+QString::number(lat,10)+" "+\
                                                   "lon: "+QString::number(lon,10)+" "+\
                                                   "alt: "+QString::number(alt,10)+"mm "+\
                                                   "eph: "+QString::number(eph,10)+" "+\
                                                   "epv: "+QString::number(epv,10)+" "+\
                                                   "vel: "+QString::number(vel,10)+"cm/s "+\
                                                   "cog: "+QString::number(cog,10)+"d "+\
                                                   "fix_type: "+fix_type_string+" " +\
                                                   "SV: "+QString::number(satellites_visible,10)+" ";
                                    msgString += QString::number(lat,10)+" "+QString::number(lon,10)+" "+QString::number(alt,10)+" "+\
                                            QString::number(eph,10)+" "+fix_type_string;
                                    emit sendMessage(24, msgString);
                                    break;
                                }
                                case JOYLINK_MSG_ID_MISSION_CURRENT:
                                {
                                    QString str = "";
                                    uint16_t seq =joylink_msg_mission_current_get_seq(&r_message);
                                    parseString += "Current mission: "+QString::number(seq,10);
                                    str = QString::number(seq,10);
                                    //qDebug()<<str;
                                    emit updateWidget1(41, str);
                                    break;
                                }

                                case JOYLINK_MSG_ID_FLY_STATUS:
                                {
                                    float airspeed = joylink_msg_fly_status_get_airspeed(&r_message); /*< Current airspeed in m/s*/
                                    float groundspeed = joylink_msg_fly_status_get_groundspeed(&r_message); /*< Current ground speed in m/s*/
                                    float alt = joylink_msg_fly_status_get_alt(&r_message); /*< Current altitude (MSL), in meters*/
                                    float climb = joylink_msg_fly_status_get_climb(&r_message); /*< Current climb rate in meters/second*/
                                    int16_t heading = joylink_msg_fly_status_get_heading(&r_message); /*< Current heading in degrees, in compass units (0..360, 0=north)*/
                                    uint16_t throttle = joylink_msg_fly_status_get_throttle(&r_message); /*< Current throttle setting in integer percent, 0 to 100*/
                                    parseString += "Fly Status>>>\r\nARSPD: "+QString::number(airspeed)+"m/s "+\
                                                   "GS: "+QString::number(groundspeed)+"m/s "+\
                                                   "alt: "+QString::number((int)alt,10)+"m "+\
                                                   "climb: "+QString::number(climb)+"m/s "+\
                                                   "heading: "+QString::number(heading,10)+" "+\
                                                   "throttle: "+QString::number(throttle,10)+"% ";
                                    msgString = QString::number(airspeed) +" "+QString::number(groundspeed)+" "+QString::number((int)alt,10)+\
                                                " "+QString::number(climb)+" "+QString::number(heading,10)+" "+QString::number(throttle,10)+"%";
                                    emit sendMessage(JOYLINK_MSG_ID_FLY_STATUS, msgString);
            //qDebug()<<parseString;
                                    break;
                                }
                                case JOYLINK_MSG_ID_SYSTEM_TIME:
                                {
                                    uint64_t time_unix_usec = joylink_msg_system_time_get_time_unix_usec(&r_message); /*< Timestamp of the master clock in microseconds since UNIX epoch.*/
                                    uint32_t time_boot_ms = joylink_msg_system_time_get_time_boot_ms(&r_message); /*< Timestamp of the component clock since boot time in milliseconds.*/
                                    parseString += "System time>>>\r\nunix "+QString::number(time_unix_usec,10)+"us "+\
                                                   "time_boot_ms: "+QString::number(time_boot_ms,10)+"ms ";
                                    break;
                                }
                                case JOYLINK_MSG_ID_CAMERA_FEEDBACK:
                                {
                                    uint16_t img_idx = joylink_msg_camera_feedback_get_img_idx(&r_message);
            //qDebug()<<"camera feedback img_index: " + QString::number(img_idx,10);
                                   parseString += "camera feedback>>>\r\n img_index: " + QString::number(img_idx,10);

                                    break;
                                }
                                case JOYLINK_MSG_ID_COMMAND_ACK:
                                {
                                    qDebug()<<"command ack...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_HOME_POSITION:
                                {
                                    qDebug()<<"home pos...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_LANDING_TARGET:
                                {
                                    qDebug()<<"landing target...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                                {
            //                        qDebug()<<"nav control output...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_POWER_STATUS:
                                {
            //                        qDebug()<<"power status...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_SCALED_PRESSURE:
                                {
            //                        qDebug()<<"scaled pressure...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_RANGEFINDER:
                                {
            //                        qDebug()<<"Rangefinder...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_MISSION_COUNT:
                                {
                                    joylink_mission_request mission_request;
                                    QString str="";
                                    this->missionCount = joylink_msg_mission_count_get_count(&r_message);
                                    if(this->missionCount==0) return;
                                    mission_request.seq = 0;
                                    mission_request.target_id = 1;
                                    mission_request.target_system = 1;
                                    joylink_msg_mission_request_send_struct(JOYLINK_COMM_1, &mission_request);
                                    str = QString::number(this->missionCount,10);
                                    emit updateWidget1(44, str);
                                    qDebug()<<"Mission count...";
                                    break;
                                }
                                case JOYLINK_MSG_ID_MISSION_REQUEST:
                                {
                                    break;
                                }
                                case JOYLINK_MSG_ID_MISSION_ELEMENT:
                                {
                                    QString str = "";
                                    QString s = "";
                                    uint16_t seq = joylink_msg_mission_element_get_seq(&r_message);
                                    float param1 = joylink_msg_mission_element_get_param1(&r_message);
                                    float param2 = joylink_msg_mission_element_get_param2(&r_message);
                                    float param3 = joylink_msg_mission_element_get_param3(&r_message);
                                    float param4 = joylink_msg_mission_element_get_param4(&r_message);
                                    float x = joylink_msg_mission_element_get_x(&r_message);
                                    float y = joylink_msg_mission_element_get_y(&r_message);
                                    float z = joylink_msg_mission_element_get_z(&r_message);
                                    uint16_t command = joylink_msg_mission_element_get_command(&r_message);
                                    uint8_t frame = joylink_msg_mission_element_get_frame(&r_message);
                                    uint8_t current = joylink_msg_mission_element_get_current(&r_message);
                                    uint8_t autocontinue = joylink_msg_mission_element_get_autocontinue(&r_message);
                                    str += QString::number(command, 10)+"   "+QString::number(seq,10)+"   "+QString::number(param1,'f', 2)+"   "+\
                                           QString::number(param2,'f', 2)+"   "+QString::number(param3,'f', 2)+"   "+QString::number(param4,'f', 2)+"   "+\
                                           QString::number(x,'f', 8)+"   "+QString::number(y,'f', 8)+"   "+QString::number(z,'f', 2)+"   "+\
                                           QString::number(frame)+"\r\n";
                                    s = QString::number(seq,10);
                                    emit updateMissionStatus1(s);
                                    if(seq>=this->missionCount-1)
                                    {
                                        joylink_mission_ack mission_ack;
                                        mission_ack.target_id = 1;
                                        mission_ack.target_system = 1;
                                        mission_ack.type = JOY_MISSION_ACCEPTED;
                                        joylink_msg_mission_ack_send_struct(JOYLINK_COMM_1, &mission_ack);
                                        s = tr("完成");
                                        emit updateMissionStatus1(s);
                                    }else
                                    {
                                        joylink_mission_request mission_request;
                                        mission_request.seq = seq +1;
                                        mission_request.target_id = 1;
                                        mission_request.target_system = 1;
                                        joylink_msg_mission_request_send_struct(JOYLINK_COMM_1, &mission_request);
                                    }
                                    emit updateTextTextEdit1(str);
                                    break;
                                }
                            default: break;
                            }
                            emit updateParseText(parseString);
                            emit updateSrcText(srcString);
                        }
                    }else if(this->getMavJoySelect()==0)
                        {
                            //qDebug()<<"mavlink";
                            if(mavlink_frame_char(MAVLINK_COMM_1, byteArray.at(i), &m_message, &m_mavlink_status)==MAVLINK_FRAMING_OK)
                            {
                                //qDebug()<<QString::number(m_message.msgid,10);
                                switch(m_message.msgid)
                                {
                                    case MAVLINK_MSG_ID_HEARTBEAT: /*#0*/
                                    {
                                        uint8_t flight_mode = mavlink_msg_heartbeat_get_custom_mode(&m_message);
                                        uint8_t base_mode = mavlink_msg_heartbeat_get_base_mode(&m_message);
                                        this->current_mode = flight_mode;
                                        QString flight_mode_string = "";
                                        switch (flight_mode)
                                        {
                                            case JOY_STABLILIZE_MODE:
                                                flight_mode_string = "STABLIZE";
                                                break;
                                            case JOY_AUTO_MODE:
                                                flight_mode_string = "AUTO";
                                                break;
                                            case JOY_GUIDED_MODE:
                                                flight_mode_string = "GUIDED";
                                                break;
                                            case JOY_RTL_MODE:
                                                flight_mode_string = "RTL";
                                                break;
                                            case JOY_LAND_MODE:
                                                flight_mode_string = "LAND";
                                                break;
                                            case JOY_POSHOLD_MODE:
                                                flight_mode_string = "POSHOLD";
                                                break;

                                            default:
                                                flight_mode_string = "UNKNOW";
                                                break;
                                        }
                                        QString arm_string = "";
                                        if(base_mode&JOY_MODE_FLAG_SAFETY_ARMED==JOY_MODE_FLAG_SAFETY_ARMED)
                                        {
                                            this->planeArm = 1;
                                            arm_string += "Arm";
                                        }else{
                                            this->planeArm = 0;
                                            arm_string += "Disarm";
                                        }

                                        emit updateWidget1(0, flight_mode_string);
                                        emit updateArmLabel(arm_string);
                                        parseString = "HeartBeart>>>\r\n fight_mode: "+flight_mode_string;
                                        msgString = flight_mode_string +" "+arm_string;
                                        emit sendMessage(0, msgString);
                                        qDebug()<<parseString;
                                        break;

                                    }

                                    case MAVLINK_MSG_ID_GPS_RAW_INT: /*#24*/
                                    {
                                        qDebug()<<"GPS RAW int";
                                        uint64_t time_usec = mavlink_msg_gps_raw_int_get_time_usec(&m_message); /* 系统运行时间  微秒 */
                                        int32_t lat = mavlink_msg_gps_raw_int_get_lat(&m_message); /* 椭球  纬度*10的7次方 */
                                        int32_t lon = mavlink_msg_gps_raw_int_get_lon(&m_message); /* 椭球  经度*10的7次方 */
                                        int32_t alt = mavlink_msg_gps_raw_int_get_alt(&m_message); /* 平均海拔高度， 注意不是椭球高度  *1000*/
                                        uint16_t eph = mavlink_msg_gps_raw_int_get_eph(&m_message); /* 水平精度 GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
                                        uint16_t epv = mavlink_msg_gps_raw_int_get_epv(&m_message); /* 垂直精度 GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
                                        uint16_t vel = mavlink_msg_gps_raw_int_get_vel(&m_message); /* GPS地速 GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
                                        uint16_t cog = mavlink_msg_gps_raw_int_get_cog(&m_message); /* Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
                                        uint8_t fix_type = mavlink_msg_gps_raw_int_get_fix_type(&m_message); /* 定位类型 See the JOY_GPS_FIX_TYPE enum.*/
                                        uint8_t satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&m_message); /* 可见卫星个数. If unknown, set to 255*/
                                        QString fix_type_string = "";
                                        if(fix_type==GPS_FIX_TYPE_NO_GPS)
                                        {
                                            fix_type_string = "NOGPS";
                                        }else if(fix_type==GPS_FIX_TYPE_NO_FIX)
                                        {
                                            fix_type_string = "NOFIX";
                                        }else if(fix_type==GPS_FIX_TYPE_2D_FIX)
                                        {
                                            fix_type_string = "2DFIX";
                                        }else if(fix_type==GPS_FIX_TYPE_3D_FIX)
                                        {
                                            fix_type_string = "3DFIX";
                                        }else if(fix_type==GPS_FIX_TYPE_DGPS)
                                        {
                                            fix_type_string = "DGPS";
                                        }

                                        parseString = "";
                                        parseString += /*"time: "+QString::number(time_usec,10)+"us\r\n"+\*/
                                                       "纬度: "+QString::number(lat,10)+"\r\n"+\
                                                       "精度: "+QString::number(lon,10)+"\r\n"+\
                                                       "高度: "+QString::number(alt,10)+"mm\r\n"+\
                                                       "水平精度: "+QString::number(eph,10)+"\r\n"+\
                                                       "航向角度: "+QString::number(cog,10)+"deg\r\n"+\
                                                       "固定类型: "+fix_type_string+"\r\n" +\
                                                       "可见卫星数: "+QString::number(satellites_visible,10)+" ";
                                        msgString = "";
                                        msgString += QString::number(lat,10)+"\r\n"+QString::number(lon,10)+"\r\n"+QString::number(alt,10)+"\r\n"+\
                                                QString::number(eph,10)+"\r\n"+fix_type_string;
                                        emit sendMessage(24, parseString);
                                        break;
                                    }

                                    case MAVLINK_MSG_ID_SYS_STATUS:
                                    {
                                         qDebug()<<"sys status";
                                        mavlink_sys_status_t sys;
                                        uint16_t voltage_battery = mavlink_msg_sys_status_get_voltage_battery(&m_message); /*< Battery voltage, in millivolts (1 = 1 millivolt)*/
                                        int16_t current_battery = mavlink_msg_sys_status_get_current_battery(&m_message); /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
                                        int8_t battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&m_message); /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/

                                        msgString = "";
                                        msgString += "电压： "+QString::number(voltage_battery,10)+"mV\r\n"+"电流： "+QString::number(current_battery*10,10)+"mA\r\n"+"电压余量："+QString::number(battery_remaining,10)+"%";
                                        emit sendMessage(1, msgString);
                                        break;
                                    }

                                     case MAVLINK_MSG_ID_COMMAND_ACK:
                                    {
                                        mavlink_command_ack_t command;
                                        command.command = mavlink_msg_command_ack_get_command(&m_message);
                                        command.result = mavlink_msg_command_ack_get_result(&m_message);
                                        msgString = "";
                                        QString tstr = "";
                                        switch ( command.result ) {

                                        case MAV_RESULT_ACCEPTED:
                                            tstr = "成功接收指令";
                                            break;
                                        case MAV_RESULT_TEMPORARILY_REJECTED:
                                            tstr = "临时拒绝";
                                            break;
                                        case MAV_RESULT_DENIED:
                                            tstr = "被拒绝";
                                            break;
                                        case MAV_RESULT_UNSUPPORTED:
                                            tstr = "指令不支持";
                                            break;
                                        case MAV_RESULT_FAILED:
                                            tstr = "指令错误";
                                            break;
                                        default:
                                            tstr = "指令错误";
                                            break;
                                        }
                                        msgString += "控制指令应答： ID->"+QString::number(command.command,10)+"  "+tstr;
                                        emit sendMessage(77, msgString);
                                        break;
                                    }
                                    case MAVLINK_MSG_ID_MAG_CAL_REPORT:
                                    {
                                        mavlink_mag_cal_report_t mag_report;
                                        float ofs_x = mavlink_msg_mag_cal_report_get_ofs_x(&m_message); /*< X offset*/
                                        float ofs_y =  mavlink_msg_mag_cal_report_get_ofs_y(&m_message); /*< Y offset*/
                                        float ofs_z =  mavlink_msg_mag_cal_report_get_ofs_z(&m_message); /*< Z offset*/
                                        uint8_t compass_id =  mavlink_msg_mag_cal_report_get_compass_id(&m_message); /*< Compass being calibrated*/
                                        uint8_t cal_mask =  mavlink_msg_mag_cal_report_get_cal_mask(&m_message); /*< Bitmask of compasses being calibrated*/
                                        uint8_t cal_status =  mavlink_msg_mag_cal_report_get_cal_status(&m_message); /*< Status (see MAG_CAL_STATUS enum)*/
                                        uint8_t autosaved =  mavlink_msg_mag_cal_report_get_autosaved(&m_message); /*< 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters*/

                                        msgString = "";
                                        msgString +="ID "+QString::number(compass_id,10)+"\r\noff_x:"+QString::number(ofs_x,'f', 2)+" off_y:"+QString::number(ofs_y,'f', 2)+" off_z:"+QString::number(ofs_z,'f', 2);
                                        if(autosaved==1) msgString += "  auto save";
                                        emit sendMessage(192, msgString);
                                        emit sendMessage(191, QString::number(compass_id,10)+QString::number(100,10));
                                        break;
                                    }

                                     case MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
                                     {
                                        uint8_t compass_id = mavlink_msg_mag_cal_progress_get_compass_id(&m_message); /*< Compass being calibrated*/
                                        uint8_t cal_mask = mavlink_msg_mag_cal_progress_get_cal_mask(&m_message); /*< Bitmask of compasses being calibrated*/
                                        uint8_t cal_status = mavlink_msg_mag_cal_progress_get_cal_status(&m_message); /*< Status (see MAG_CAL_STATUS enum)*/
                                        uint8_t attempt = mavlink_msg_mag_cal_progress_get_attempt(&m_message); /*< Attempt number*/
                                        uint8_t completion_pct = mavlink_msg_mag_cal_progress_get_completion_pct(&m_message); /*< Completion percentage*/

                                        emit sendMessage(191, QString::number(compass_id,10)+QString::number(completion_pct,10));
                                        break;
                                     }

                                    case MAVLINK_MSG_ID_VFR_HUD:
                                    {
                                        //float airspeed; /*< Current airspeed in m/s*/
                                        float groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&m_message); /*< Current ground speed in m/s*/
                                        float alt = mavlink_msg_vfr_hud_get_alt(&m_message); /*< Current altitude (MSL), in meters*/
                                        //float climb; /*< Current climb rate in meters/second*/
                                        int16_t heading = mavlink_msg_vfr_hud_get_heading(&m_message); /*< Current heading in degrees, in compass units (0..360, 0=north)*/
                                        uint16_t throttle = mavlink_msg_vfr_hud_get_throttle(&m_message); /*< Current throttle setting in integer percent, 0 to 100*/

                                        msgString = "";
                                        msgString +="高度 "+QString::number(alt,'f',2)+"米\r\n地速 "+QString::number(groundspeed, 'f', 2)+\
                                                    "m/s\r\n航向 "+QString::number(heading, 10)+"\r\n油门 "+QString::number(throttle, 10)+"%";

                                        emit sendMessage(74, msgString);
                                        break;
                                    }

                                    case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
                                    {
                                        //float velocity_variance; /*< Velocity variance*/
                                        //float pos_horiz_variance; /*< Horizontal Position variance*/
                                        //float pos_vert_variance; /*< Vertical Position variance*/
                                        float compass_variance = mavlink_msg_ekf_status_report_get_compass_variance(&m_message); /*< Compass variance*/
                                        //float terrain_alt_variance; /*< Terrain Altitude variance*/

                                        msgString = "";
                                        msgString +="Compass healthy: "+QString::number(compass_variance,'f',2);
                                        emit sendMessage(193, msgString);

                                        break;
                                    }

                                    default: break;

                               }
                               //qDebug()<<parseString;
                            emit updateParseText(parseString);
                            emit updateSrcText(srcString);
                        }
                    }
               }
            }
         this->msleep(50);
        }
    }
}

void SerialRecThread::setRunFlag(RunFlag flag)
{
    if(flag>1||flag<0) return;
    this->runFlag = flag;
}

RunFlag SerialRecThread::getRunFlag()
{
    return this->runFlag;
}

QString SerialRecThread::charToHex(quint8 c)
{
    QString str = "";
    quint8  high = c/16;
    quint8  low = c%16;

    if(high<10) str+=QString::number(high, 10);
    else if(high==10) str+="A";
    else if(high==11) str+="B";
    else if(high==12) str+="C";
    else if(high==13) str+="D";
    else if(high==14) str+="E";
    else if(high==15) str+="F";
    if(low<10) str+=QString::number(low, 10);
    else if(low==10) str+="A";
    else if(low==11) str+="B";
    else if(low==12) str+="C";
    else if(low==13) str+="D";
    else if(low==14) str+="E";
    else if(low==15) str+="F";
    return str;
}

void SerialRecThread::setSerialPort(QSerialPort *pserialPort)
{
    if(pserialPort!=NULL)
    {
        this->serialPort = pserialPort;
    }
}

char SerialRecThread::getMavJoySelect()
{
    return this->mavJoySelect;
}

void SerialRecThread::setMavJoySelect(char flag)
{
    if(flag>1) return;
    this->mavJoySelect = flag;
}

void SerialRecThread::setLocation(float lat, float lng, int32_t alt)
{
    if(lat>90 || lng>180 || alt>5000) return;
    this->location.lat = lat;
    this->location.lng = lng;
    this->location.alt = alt;
}
