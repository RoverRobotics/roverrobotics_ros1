#pragma once

#include "protocol_base.hpp"

namespace RoverRobotics {
class ProProtocolObject;
}
class RoverRobotics::ProProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  ProProtocolObject(const char* device, std::string new_comm_type,
                    bool closed_loop, PidGains pid);
  // ~ProProtocolObject() override;
  void update_drivetrim(double) override;
  void send_estop(bool) override;
  statusData status_request() override;
  statusData info_request() override;
  void send_speed(double*) override;
  void unpack_comm_response(std::vector<uint32_t>) override;
  bool isConnected() override;
  void register_comm_base(const char* device) override;
  void sendCommand(int sleeptime, std::vector<uint32_t> datalist);

 private:
  const float MOTOR_RPM_TO_MPS_RATIO = 13749 / 1.26;
  const float MOTOR_RPM_TO_MPS_CFB = -0.07;
  const int MOTOR_NEUTRAL = 125;
  const int MOTOR_MAX = 250;
  const int MOTOR_MIN = 0;
  const int startbyte = 253;
  const int requestbyte = 10;
  const int baudrate = 4097;
  const int RECEIVE_MSG_LEN = 5;
  // const int commandbit = 20;
  std::unique_ptr<CommBase> comm_base;
  std::string comm_type;

  std::mutex writemutex;
  statusData robotstatus_;
  int motors_speeds_[3];
  double trimvalue;
  std::thread writethread;
  std::thread writethread2;
  bool estop_;
  // Motor PID variables
  OdomControl motor1_control;
  OdomControl motor2_control;
  bool closed_loop_;
  PidGains pid_;
  std::chrono::steady_clock::time_point motor1_prev_t;
  std::chrono::steady_clock::time_point motor2_prev_t;

  enum robot_motors{
    LEFT_MOTOR,
    RIGHT_MOTOR,
    FLIPPER_MOTOR
  };

  enum uart_param {
    REG_PWR_TOTAL_CURRENT,
    REG_MOTOR_FB_RPM_LEFT = 2,
    REG_MOTOR_FB_RPM_RIGHT = 4,
    REG_FLIPPER_FB_POSITION_POT1 = 6,
    REG_FLIPPER_FB_POSITION_POT2 = 8,
    REG_MOTOR_FB_CURRENT_LEFT = 10,
    REG_MOTOR_FB_CURRENT_RIGHT = 12,
    REG_MOTOR_ENCODER_COUNT_LEFT = 14,
    REG_MOTOR_ENCODER_COUNT_RIGHT = 16,
    REG_MOTOR_FAULT_FLAG_LEFT = 18,
    REG_MOTOR_TEMP_LEFT = 20,
    REG_MOTOR_TEMP_RIGHT = 22,
    REG_PWR_BAT_VOLTAGE_A = 24,
    REG_PWR_BAT_VOLTAGE_B = 26,
    EncoderInterval_0 = 28,
    EncoderInterval_1 = 30,
    EncoderInterval_2 = 32,
    REG_ROBOT_REL_SOC_A = 34,
    REG_ROBOT_REL_SOC_B = 36,
    REG_MOTOR_CHARGER_STATE = 38,
    BuildNO = 40,
    REG_PWR_A_CURRENT = 42,
    REG_PWR_B_CURRENT = 44,
    REG_MOTOR_FLIPPER_ANGLE = 46,
    to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48,
    to_computer_REG_MOTOR_SLOW_SPEED = 50,
    BATTERY_STATUS_A = 52,
    BATTERY_STATUS_B = 54,
    BATTERY_MODE_A = 56,
    BATTERY_MODE_B = 58,
    BATTERY_TEMP_A = 60,
    BATTERY_TEMP_B = 62,
    BATTERY_VOLTAGE_A = 64,
    BATTERY_VOLTAGE_B = 66,
    BATTERY_CURRENT_A = 68,
    BATTERY_CURRENT_B = 70
  };
};
