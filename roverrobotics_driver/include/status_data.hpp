#pragma once
struct statusData {
  // Motor Infos
  int motor1_id;
  float motor1_rpm;
  float motor1_current;
  float motor1_temp;
  float motor1_mos_temp;
  int motor2_id;
  float motor2_rpm;
  float motor2_current;
  float motor2_temp;
  float motor2_mos_temp;
  int motor3_id;
  float motor3_rpm;
  float motor3_current;
  float motor3_temp;
  float motor3_mos_temp;
  int motor4_id;
  float motor4_rpm;
  float motor4_current;
  float motor4_temp;
  float motor4_mos_temp;

  // Battery Infos
  float battery1_voltage;
  float battery2_voltage;
  float battery1_temp;
  float battery2_temp;
  float battery1_current;
  float battery2_current;
  float battery1_SOC;
  float battery2_SOC;
  float battery1_fault_flag;
  float battery2_fault_flag;

  // Robot Infos
  float robot_guid;
  float robot_firmware;
  float robot_fault_flag;
  float robot_fan_speed;
  float robot_speed_limit;

  // Flipper Infos
  float motor3_angle;
  float motor3_sensor1;
  float motor3_sensor2;
};