#pragma once
#include "comm_can.hpp"
#include "comm_base.hpp"
#include "protocol_base.hpp"
#include "comm_serial.hpp"
namespace RoverRobotics {
class ZeroProtocolObject;
}
class RoverRobotics::ZeroProtocolObject
    : public RoverRobotics::BaseProtocolObject {
 public:
  ZeroProtocolObject(const char* device, std::string new_comm_type,bool closed_loop, PidGains pid);
  ~ZeroProtocolObject() override;
  void update_drivetrim(double) override;
  void translate_send_estop(bool) override;
  statusData translate_send_robot_status_request() override;
  statusData translate_send_robot_info_request() override;
  void translate_send_speed(double*) override;
  void unpack_robot_response(unsigned char *) override;
  bool isConnected() override;
  // void register_state_response_cb(boost::function<int(void)> _f);
  void register_comm_base(const char* device) override;

 private:
  double trimvalue;
  std::unique_ptr<CommBase> comm_base;
  // comm_base_t comm_base;
  // mutex comm_base_mutex;
  void (*state_response_cb_function)();
};
