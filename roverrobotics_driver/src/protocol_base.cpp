#include "protocol_base.hpp"
// only use this to declare shared functions for all robot protocols.

RoverRobotics::BaseProtocolObject::BaseProtocolObject() {}
RoverRobotics::BaseProtocolObject::~BaseProtocolObject() {}
// void update_drivetrim(double) {}
// void translate_send_estop() {}
// void translate_send_speed(double linearx, double angularz) {}
// statusData translate_send_robot_status_request() {}
// robotInfo translate_send_robot_info_request() {}
// void handle_unsupported_ros_message() {}
// void unpack_robot_response() {}
// bool isConnected() {}
// // virtual void register_state_response_cb(boost::function<int(void)> _f);
// void register_comm_base() {}