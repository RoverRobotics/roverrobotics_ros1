#include "protocol_pro.hpp"

namespace RoverRobotics {
ProProtocolObject::ProProtocolObject(const char* device,
                                     std::string new_comm_type) {
  comm_type = new_comm_type;
  register_comm_base(device);
}

ProProtocolObject::~ProProtocolObject() {
  // Decontructor
}

void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }

void ProProtocolObject::translate_send_estop() {
  motors_speeds_[0] = MOTOR_NEUTRAL;
  motors_speeds_[1] = MOTOR_NEUTRAL;
  motors_speeds_[2] = MOTOR_NEUTRAL;
}

statusData ProProtocolObject::translate_send_robot_status_request() {
  sendCommand(10, 4);
  output.motor1_rpm = 808;
  // atof(read_buffer[0]); //convert char* to float from buffer
  unpack_robot_response();
  return output;
}

robotInfo ProProtocolObject::translate_send_robot_info_request() {
  // TODO:
}

void ProProtocolObject::translate_send_speed(double* controlarray) {
  double turn_rate = controlarray[1];
  double linear_rate = controlarray[0];
  double flipper_rate = controlarray[2];
  // apply trim value
  if (turn_rate == 0) {
    if (linear_rate > 0) {
      turn_rate = trimvalue;
    } else if (linear_rate < 0) {
      turn_rate = -trimvalue;
    }
  }

  double diff_vel_commanded = turn_rate;
  motors_speeds_[0] = linear_rate - 0.5 * diff_vel_commanded;
  motors_speeds_[1] = linear_rate + 0.5 * diff_vel_commanded;
  motors_speeds_[2] = controlarray[2];
  sendCommand(10, 4);
}

void ProProtocolObject::handle_unsupported_ros_message() {
  // TODO: TBD
}

void ProProtocolObject::unpack_robot_response() {
  int data = 0;
  //* Placeholder; While not getting enough data
  // while (data < -1) {                     // TODO
    std::cout << (uint8_t*)comm_base->readfromdevice() << std::endl;  // Get data
  // }
}

bool ProProtocolObject::isConnected() {
  return true;
  // TODO: TBD
}

void ProProtocolObject::register_comm_base(const char* device) {
  if (comm_type == "serial") {
    comm_base = std::make_unique<CommSerial>(device);
  } else if (comm_type == "can") {
    comm_base = std::make_unique<CanManager>(device);
  }
}

bool ProProtocolObject::sendCommand(int param1, int param2) {
  // Param 1: 10 to get data, 240 for low speed mode
  if (comm_type == "serial") {
    write_buffer[0] = 253;
    write_buffer[1] = (unsigned char)motors_speeds_[0];  // left motor
    write_buffer[2] = (unsigned char)motors_speeds_[1];  // right motor
    write_buffer[3] = (unsigned char)motors_speeds_[2];  // flipper
    write_buffer[4] = (unsigned char)param1;
    write_buffer[5] = (unsigned char)param2;  // Param 2:
    // Calculate Checksum
    write_buffer[6] =
        (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] +
                     write_buffer[4] + write_buffer[5]) %
                        255;
    comm_base->writetodevice(write_buffer);
  } else if (comm_type == "can") {
    return false;  //* no CAN for rover pro yet
  } else {         //! How did you get here?
    return false;  // TODO: Return error ?
  }
}

}  // namespace RoverRobotics
