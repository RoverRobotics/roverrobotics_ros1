#pragma once

#include "protocol_base.hpp"

namespace RoverRobotics {
class TerrapinProtocolObject;
}
class RoverRobotics::TerrapinProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  TerrapinProtocolObject(const char* device, std::string new_comm_type,
                         bool closed_loop, PidGains pid, int motors_id[]);
  // ~TerrapinProtocolObject() override;
  void update_drivetrim(double) override;
  void translate_send_estop(bool) override;
  statusData translate_send_robot_status_request() override;
  statusData translate_send_robot_info_request() override;
  void translate_send_speed(double*) override;
  void unpack_comm_response(std::vector<uint32_t>) override;
  bool isConnected() override;
  void register_comm_base(const char* device) override;
  void sendCommand(int sleeptime, std::vector<uint32_t> datalist);
  float clip(float n, float lower, float upper);

 private:
  const float MOTOR_RPM_TO_MPS_RATIO = 13749 / 1.26;
  const float MOTOR_RPM_TO_MPS_CFB = -0.07;
  const int MOTOR_NEUTRAL = 0;
  std::unique_ptr<CommBase> comm_base;
  std::string comm_type;
  struct can_frame frame;
  std::mutex writemutex;
  statusData robotstatus_;
  int motors_id_[4];
  int motors_speeds_[4];
  double trimvalue;
  unsigned char write_buffer[7];
  char* read_buffer[7];
  std::thread writethread;
  std::thread writethread2;
  bool estop_;
  // Motor PID variables
  OdomControl motor1_control;
  OdomControl motor2_control;
  OdomControl motor3_control;
  OdomControl motor4_control;
  bool closed_loop_;
  PidGains pid_;
  std::chrono::steady_clock::time_point motor1_prev_t;
  std::chrono::steady_clock::time_point motor2_prev_t;
};
