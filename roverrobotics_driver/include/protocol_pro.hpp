#pragma once
#include <stdio.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

#include "comm_base.hpp"
#include "comm_can.hpp"
#include "comm_serial.hpp"
#include "protocol_base.hpp"

namespace RoverRobotics {
class ProProtocolObject;
}
class RoverRobotics::ProProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  ProProtocolObject(const char* device, std::string new_comm_type,
                    bool closed_loop, PidGains pid);
  ~ProProtocolObject() override;
  void update_drivetrim(double) override;
  void translate_send_estop(bool) override;
  statusData translate_send_robot_status_request() override;
  statusData translate_send_robot_info_request() override;
  void translate_send_speed(double*) override;
  void unpack_robot_response(unsigned char*) override;
  bool isConnected() override;
  // void register_state_response_cb(boost::function<int(void)> _f);
  void register_comm_base(const char* device) override;
  void sendCommand(int sleeptime, std::vector<int> datalist);

 private:
  const int MOTOR_NEUTRAL = 125;
  std::unique_ptr<CommBase> comm_base;
  std::string comm_type;

  std::mutex writemutex;
  statusData robotstatus_;
  int motors_speeds_[3];
  double trimvalue;
  unsigned char write_buffer[7];
  char* read_buffer[7];
  std::thread writethread;
  std::thread writethread2;
  bool estop_;
  //Motor PID variables
  OdomControl motor1_control;
  OdomControl motor2_control;
  bool closed_loop_;
  PidGains pid_;
  std::chrono::steady_clock::time_point motor1_prev_t;
  std::chrono::steady_clock::time_point motor2_prev_t;
  // mutex comm_base_mutex;
  void (*state_response_cb_function)();
};
