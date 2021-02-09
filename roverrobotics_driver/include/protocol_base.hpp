#pragma once

#include "comm_base.hpp"
#include "comm_can.hpp"
#include "comm_serial.hpp"

namespace RoverRobotics {
class BaseProtocolObject;
}
class RoverRobotics::BaseProtocolObject {
 public:
  BaseProtocolObject() = default;
  virtual ~BaseProtocolObject();
  virtual void update_drivetrim(double) = 0;
  virtual void translate_send_estop(bool) = 0;
  virtual void translate_send_speed(double*) = 0;
  virtual statusData translate_send_robot_status_request() = 0;
  virtual statusData translate_send_robot_info_request() = 0;
  // virtual void unpack_serial(unsigned char*) = 0;
  // virtual void unpack_can(struct can_frame) = 0;
  virtual void unpack_comm_response(std::vector<uint32_t>) = 0;
  virtual bool isConnected() = 0;
  // virtual void register_state_response_cb(boost::function<int(void)> _f);
  virtual void register_comm_base(const char* device) = 0;
};
