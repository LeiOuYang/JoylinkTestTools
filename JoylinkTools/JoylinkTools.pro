#-------------------------------------------------
#
# Project created by QtCreator 2018-05-19T17:55:26
#
#-------------------------------------------------

QT       += core gui
QT += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = JoylinkTools
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    SerialRecThread.cpp \
    serialsendthread.cpp \
    JoylinkLib/LoopQueue.c

HEADERS += \
    mainwindow.h \
    SerialRecThread.h \
    JoylinkLib/Joylink.h \
    serialsendthread.h \
    JoylinkLib/LoopQueue.h \
    MavlinkV1/ardupilotmega/ardupilotmega.h \
    MavlinkV1/ardupilotmega/mavlink_msg_ahrs.h \
    MavlinkV1/ardupilotmega/mavlink_msg_ahrs2.h \
    MavlinkV1/ardupilotmega/mavlink_msg_ahrs3.h \
    MavlinkV1/ardupilotmega/mavlink_msg_airspeed_autocal.h \
    MavlinkV1/ardupilotmega/mavlink_msg_ap_adc.h \
    MavlinkV1/ardupilotmega/mavlink_msg_autopilot_version_request.h \
    MavlinkV1/ardupilotmega/mavlink_msg_battery2.h \
    MavlinkV1/ardupilotmega/mavlink_msg_camera_feedback.h \
    MavlinkV1/ardupilotmega/mavlink_msg_camera_status.h \
    MavlinkV1/ardupilotmega/mavlink_msg_compassmot_status.h \
    MavlinkV1/ardupilotmega/mavlink_msg_data16.h \
    MavlinkV1/ardupilotmega/mavlink_msg_data32.h \
    MavlinkV1/ardupilotmega/mavlink_msg_data64.h \
    MavlinkV1/ardupilotmega/mavlink_msg_data96.h \
    MavlinkV1/ardupilotmega/mavlink_msg_digicam_configure.h \
    MavlinkV1/ardupilotmega/mavlink_msg_digicam_control.h \
    MavlinkV1/ardupilotmega/mavlink_msg_ekf_status_report.h \
    MavlinkV1/ardupilotmega/mavlink_msg_fence_fetch_point.h \
    MavlinkV1/ardupilotmega/mavlink_msg_fence_point.h \
    MavlinkV1/ardupilotmega/mavlink_msg_fence_status.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gcs_control_code.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gimbal_control.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gimbal_report.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gimbal_torque_cmd_report.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gopro_get_request.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gopro_get_response.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gopro_heartbeat.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gopro_set_request.h \
    MavlinkV1/ardupilotmega/mavlink_msg_gopro_set_response.h \
    MavlinkV1/ardupilotmega/mavlink_msg_hwstatus.h \
    MavlinkV1/ardupilotmega/mavlink_msg_led_control.h \
    MavlinkV1/ardupilotmega/mavlink_msg_limits_status.h \
    MavlinkV1/ardupilotmega/mavlink_msg_mag_cal_progress.h \
    MavlinkV1/ardupilotmega/mavlink_msg_mag_cal_report.h \
    MavlinkV1/ardupilotmega/mavlink_msg_meminfo.h \
    MavlinkV1/ardupilotmega/mavlink_msg_mount_configure.h \
    MavlinkV1/ardupilotmega/mavlink_msg_mount_control.h \
    MavlinkV1/ardupilotmega/mavlink_msg_mount_status.h \
    MavlinkV1/ardupilotmega/mavlink_msg_pid_tuning.h \
    MavlinkV1/ardupilotmega/mavlink_msg_radio.h \
    MavlinkV1/ardupilotmega/mavlink_msg_rally_fetch_point.h \
    MavlinkV1/ardupilotmega/mavlink_msg_rally_point.h \
    MavlinkV1/ardupilotmega/mavlink_msg_rangefinder.h \
    MavlinkV1/ardupilotmega/mavlink_msg_remote_log_block_status.h \
    MavlinkV1/ardupilotmega/mavlink_msg_remote_log_data_block.h \
    MavlinkV1/ardupilotmega/mavlink_msg_rpm.h \
    MavlinkV1/ardupilotmega/mavlink_msg_sensor_offsets.h \
    MavlinkV1/ardupilotmega/mavlink_msg_set_mag_offsets.h \
    MavlinkV1/ardupilotmega/mavlink_msg_simstate.h \
    MavlinkV1/ardupilotmega/mavlink_msg_wind.h \
    MavlinkV1/ardupilotmega/testsuite.h \
    MavlinkV1/common/common.h \
    MavlinkV1/common/mavlink.h \
    MavlinkV1/common/mavlink_msg_actuator_control_target.h \
    MavlinkV1/common/mavlink_msg_adsb_vehicle.h \
    MavlinkV1/common/mavlink_msg_altitude.h \
    MavlinkV1/common/mavlink_msg_att_pos_mocap.h \
    MavlinkV1/common/mavlink_msg_attitude.h \
    MavlinkV1/common/mavlink_msg_attitude_quaternion.h \
    MavlinkV1/common/mavlink_msg_attitude_quaternion_cov.h \
    MavlinkV1/common/mavlink_msg_attitude_target.h \
    MavlinkV1/common/mavlink_msg_auth_key.h \
    MavlinkV1/common/mavlink_msg_autopilot_version.h \
    MavlinkV1/common/mavlink_msg_battery_status.h \
    MavlinkV1/common/mavlink_msg_camera_trigger.h \
    MavlinkV1/common/mavlink_msg_change_operator_control.h \
    MavlinkV1/common/mavlink_msg_change_operator_control_ack.h \
    MavlinkV1/common/mavlink_msg_collision.h \
    MavlinkV1/common/mavlink_msg_command_ack.h \
    MavlinkV1/common/mavlink_msg_command_int.h \
    MavlinkV1/common/mavlink_msg_command_long.h \
    MavlinkV1/common/mavlink_msg_control_system_state.h \
    MavlinkV1/common/mavlink_msg_data_stream.h \
    MavlinkV1/common/mavlink_msg_data_transmission_handshake.h \
    MavlinkV1/common/mavlink_msg_debug.h \
    MavlinkV1/common/mavlink_msg_debug_vect.h \
    MavlinkV1/common/mavlink_msg_distance_sensor.h \
    MavlinkV1/common/mavlink_msg_encapsulated_data.h \
    MavlinkV1/common/mavlink_msg_estimator_status.h \
    MavlinkV1/common/mavlink_msg_extended_sys_state.h \
    MavlinkV1/common/mavlink_msg_file_transfer_protocol.h \
    MavlinkV1/common/mavlink_msg_follow_target.h \
    MavlinkV1/common/mavlink_msg_global_position_int.h \
    MavlinkV1/common/mavlink_msg_global_position_int_cov.h \
    MavlinkV1/common/mavlink_msg_global_vision_position_estimate.h \
    MavlinkV1/common/mavlink_msg_gps2_raw.h \
    MavlinkV1/common/mavlink_msg_gps2_rtk.h \
    MavlinkV1/common/mavlink_msg_gps_global_origin.h \
    MavlinkV1/common/mavlink_msg_gps_inject_data.h \
    MavlinkV1/common/mavlink_msg_gps_input.h \
    MavlinkV1/common/mavlink_msg_gps_raw_int.h \
    MavlinkV1/common/mavlink_msg_gps_rtcm_data.h \
    MavlinkV1/common/mavlink_msg_gps_rtk.h \
    MavlinkV1/common/mavlink_msg_gps_status.h \
    MavlinkV1/common/mavlink_msg_heartbeat.h \
    MavlinkV1/common/mavlink_msg_high_latency.h \
    MavlinkV1/common/mavlink_msg_highres_imu.h \
    MavlinkV1/common/mavlink_msg_hil_actuator_controls.h \
    MavlinkV1/common/mavlink_msg_hil_controls.h \
    MavlinkV1/common/mavlink_msg_hil_gps.h \
    MavlinkV1/common/mavlink_msg_hil_optical_flow.h \
    MavlinkV1/common/mavlink_msg_hil_rc_inputs_raw.h \
    MavlinkV1/common/mavlink_msg_hil_sensor.h \
    MavlinkV1/common/mavlink_msg_hil_state.h \
    MavlinkV1/common/mavlink_msg_hil_state_quaternion.h \
    MavlinkV1/common/mavlink_msg_home_position.h \
    MavlinkV1/common/mavlink_msg_landing_target.h \
    MavlinkV1/common/mavlink_msg_local_position_ned.h \
    MavlinkV1/common/mavlink_msg_local_position_ned_cov.h \
    MavlinkV1/common/mavlink_msg_local_position_ned_system_global_offset.h \
    MavlinkV1/common/mavlink_msg_log_data.h \
    MavlinkV1/common/mavlink_msg_log_entry.h \
    MavlinkV1/common/mavlink_msg_log_erase.h \
    MavlinkV1/common/mavlink_msg_log_request_data.h \
    MavlinkV1/common/mavlink_msg_log_request_end.h \
    MavlinkV1/common/mavlink_msg_log_request_list.h \
    MavlinkV1/common/mavlink_msg_manual_control.h \
    MavlinkV1/common/mavlink_msg_manual_setpoint.h \
    MavlinkV1/common/mavlink_msg_memory_vect.h \
    MavlinkV1/common/mavlink_msg_message_interval.h \
    MavlinkV1/common/mavlink_msg_mission_ack.h \
    MavlinkV1/common/mavlink_msg_mission_clear_all.h \
    MavlinkV1/common/mavlink_msg_mission_count.h \
    MavlinkV1/common/mavlink_msg_mission_current.h \
    MavlinkV1/common/mavlink_msg_mission_item.h \
    MavlinkV1/common/mavlink_msg_mission_item_int.h \
    MavlinkV1/common/mavlink_msg_mission_item_reached.h \
    MavlinkV1/common/mavlink_msg_mission_request.h \
    MavlinkV1/common/mavlink_msg_mission_request_int.h \
    MavlinkV1/common/mavlink_msg_mission_request_list.h \
    MavlinkV1/common/mavlink_msg_mission_request_partial_list.h \
    MavlinkV1/common/mavlink_msg_mission_set_current.h \
    MavlinkV1/common/mavlink_msg_mission_write_partial_list.h \
    MavlinkV1/common/mavlink_msg_named_value_float.h \
    MavlinkV1/common/mavlink_msg_named_value_int.h \
    MavlinkV1/common/mavlink_msg_nav_controller_output.h \
    MavlinkV1/common/mavlink_msg_optical_flow.h \
    MavlinkV1/common/mavlink_msg_optical_flow_rad.h \
    MavlinkV1/common/mavlink_msg_param_map_rc.h \
    MavlinkV1/common/mavlink_msg_param_request_list.h \
    MavlinkV1/common/mavlink_msg_param_request_read.h \
    MavlinkV1/common/mavlink_msg_param_set.h \
    MavlinkV1/common/mavlink_msg_param_value.h \
    MavlinkV1/common/mavlink_msg_ping.h \
    MavlinkV1/common/mavlink_msg_position_target_global_int.h \
    MavlinkV1/common/mavlink_msg_position_target_local_ned.h \
    MavlinkV1/common/mavlink_msg_power_status.h \
    MavlinkV1/common/mavlink_msg_radio_status.h \
    MavlinkV1/common/mavlink_msg_raw_imu.h \
    MavlinkV1/common/mavlink_msg_raw_pressure.h \
    MavlinkV1/common/mavlink_msg_rc_channels.h \
    MavlinkV1/common/mavlink_msg_rc_channels_override.h \
    MavlinkV1/common/mavlink_msg_rc_channels_raw.h \
    MavlinkV1/common/mavlink_msg_rc_channels_scaled.h \
    MavlinkV1/common/mavlink_msg_request_data_stream.h \
    MavlinkV1/common/mavlink_msg_resource_request.h \
    MavlinkV1/common/mavlink_msg_safety_allowed_area.h \
    MavlinkV1/common/mavlink_msg_safety_set_allowed_area.h \
    MavlinkV1/common/mavlink_msg_scaled_imu.h \
    MavlinkV1/common/mavlink_msg_scaled_imu2.h \
    MavlinkV1/common/mavlink_msg_scaled_imu3.h \
    MavlinkV1/common/mavlink_msg_scaled_pressure.h \
    MavlinkV1/common/mavlink_msg_scaled_pressure2.h \
    MavlinkV1/common/mavlink_msg_scaled_pressure3.h \
    MavlinkV1/common/mavlink_msg_serial_control.h \
    MavlinkV1/common/mavlink_msg_servo_output_raw.h \
    MavlinkV1/common/mavlink_msg_set_actuator_control_target.h \
    MavlinkV1/common/mavlink_msg_set_attitude_target.h \
    MavlinkV1/common/mavlink_msg_set_gps_global_origin.h \
    MavlinkV1/common/mavlink_msg_set_home_position.h \
    MavlinkV1/common/mavlink_msg_set_mode.h \
    MavlinkV1/common/mavlink_msg_set_position_target_global_int.h \
    MavlinkV1/common/mavlink_msg_set_position_target_local_ned.h \
    MavlinkV1/common/mavlink_msg_sim_state.h \
    MavlinkV1/common/mavlink_msg_statustext.h \
    MavlinkV1/common/mavlink_msg_sys_status.h \
    MavlinkV1/common/mavlink_msg_system_time.h \
    MavlinkV1/common/mavlink_msg_terrain_check.h \
    MavlinkV1/common/mavlink_msg_terrain_data.h \
    MavlinkV1/common/mavlink_msg_terrain_report.h \
    MavlinkV1/common/mavlink_msg_terrain_request.h \
    MavlinkV1/common/mavlink_msg_timesync.h \
    MavlinkV1/common/mavlink_msg_v2_extension.h \
    MavlinkV1/common/mavlink_msg_vfr_hud.h \
    MavlinkV1/common/mavlink_msg_vibration.h \
    MavlinkV1/common/mavlink_msg_vicon_position_estimate.h \
    MavlinkV1/common/mavlink_msg_vision_position_estimate.h \
    MavlinkV1/common/mavlink_msg_vision_speed_estimate.h \
    MavlinkV1/common/mavlink_msg_wind_cov.h \
    MavlinkV1/common/testsuite.h \
    MavlinkV1/common/version.h \
    MavlinkV1/uAvionix/testsuite.h \
    MavlinkV1/uAvionix/uAvionix.h \
    MavlinkV1/checksum.h \
    MavlinkV1/mavlink_conversions.h \
    MavlinkV1/mavlink_helpers.h \
    MavlinkV1/mavlink_helpers.h~RF7e965b5.TMP \
    MavlinkV1/mavlink_helpers.h~RF7fd7880.TMP \
    MavlinkV1/mavlink_helpers.h~RF8030f0b.TMP \
    MavlinkV1/mavlink_helpers.h~RF81ed40d.TMP \
    MavlinkV1/mavlink_helpers.h~RFa7508cd.TMP \
    MavlinkV1/mavlink_helpers.h~RFdb122d.TMP \
    MavlinkV1/mavlink_types.h \
    MavlinkV1/protocol.h \
    MavlinkV1/user_pixhawk_type.h

FORMS += \
        mainwindow.ui

SUBDIRS += \
    JoylinkTools.pro

