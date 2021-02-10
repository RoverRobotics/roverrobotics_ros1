
#include "protocol_zero.hpp"

namespace RoverRobotics {
ZeroProtocolObject::ZeroProtocolObject(const char* device,
                                       std::string new_comm_type,
                                       bool closed_loop, PidGains pid) {
//   comm_type == new_comm_type;
//   closed_loop_ = closed_loop;
//   estop_ = false;
//   motors_speed_[0] = 0;
//   motors_speed_[1] = 0;
//   motors_speed_[2] = 0;
//   motors_speed_[3] = 0;
}

void ZeroProtocolObject::update_drivetrim(double) {}

void ZeroProtocolObject::send_estop(bool) {}

statusData ZeroProtocolObject::status_request() {}

statusData ZeroProtocolObject::info_request() {}

void ZeroProtocolObject::send_speed(double*) {}

void ZeroProtocolObject::unpack_comm_response(std::vector<uint32_t> ) {}

bool ZeroProtocolObject::isConnected() {}

void ZeroProtocolObject::register_comm_base(const char* device) {}

}  // namespace RoverRobotics