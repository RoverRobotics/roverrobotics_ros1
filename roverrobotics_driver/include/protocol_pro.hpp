#pragma once
#include <stdio.h>

#include <mutex>

#include "comm_base.hpp"
#include "comm_can.hpp"
#include "comm_serial.hpp"
#include "protocol_base.hpp"
#include "robot_info.hpp"
#include "status_data.hpp"
#include <cmath>
namespace RoverRobotics {
class ProProtocolObject;
}
class RoverRobotics::ProProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  ProProtocolObject(const char* device, std::string new_comm_type);
  ~ProProtocolObject() override;
  void update_drivetrim(double) override;
  void translate_send_estop() override;
  statusData translate_send_robot_status_request() override;
  robotInfo translate_send_robot_info_request() override;
  void translate_send_speed(double*) override;
  void handle_unsupported_ros_message() override;
  void unpack_robot_response(unsigned char *) override;
  bool isConnected() override;
  // void register_state_response_cb(boost::function<int(void)> _f);
  void register_comm_base(const char* device) override;
  void sendCommand(int sleeptime,const int * datalist);

 private:
  
  const int MOTOR_NEUTRAL = 125;
  std::unique_ptr<CommBase> comm_base;
  std::string comm_type;

  std::mutex writemutex;
  statusData robotstatus_;
  robotInfo robotinfo_;
  int motors_speeds_[3];
  double trimvalue;
  unsigned char write_buffer[7];
  char* read_buffer[7];
  std::thread writethread;
  const int fast_data[4] = {2 ,4 , 28,30};
  const int slow_data[7] = {10 , 12, 20, 22, 38, 40, 64};


  // mutex comm_base_mutex;
  void (*state_response_cb_function)();

  };
  
