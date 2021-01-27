#include "protocol_pro.hpp"


namespace RoverRobotics {
ProProtocolObject::ProProtocolObject(const char* device,
                                     std::string new_comm_type) {
  comm_type = new_comm_type;
  // if (comm_type == "serial") {
  //   // comm_base = new CommSerial(device); //bugged
  //   comm_base = std::make_unique<CommSerial>(device);
  // } else if (comm_type == "can") {
  //   comm_base = std::make_unique<CanManager>(device);
  // }
  // comm_base = new CommSerial(device); //THIS SHOULD"VE work
  // comm_base.reset(new CommSerial(device));
  // comm_base = std::make_unique<CommSerial>(device);
  comm_base = std::unique_ptr<CommSerial>(new CommSerial(device));
}
ProProtocolObject::~ProProtocolObject() {}
void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }
void ProProtocolObject::translate_send_estop() {
  if (comm_type == "serial") {
    const int MOTOR_NEUTRAL = 125;
    write_buffer[0] = 253;
    write_buffer[1] = (unsigned char)MOTOR_NEUTRAL;  // left motor
    write_buffer[2] = (unsigned char)MOTOR_NEUTRAL;  // right motor
    write_buffer[3] = (unsigned char)MOTOR_NEUTRAL;  // flipper
    // write_buffer[4] = (unsigned char)param1;         // Param 1: 10 to get
    // data, 240 for low speed mode
    // write_buffer[5] = (unsigned char)param2;         // Param 2:
    // Calculate Checksum
    write_buffer[6] =
        (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] +
                     write_buffer[4] + write_buffer[5]) %
                        255;
    comm_base->writetodevice(write_buffer); //BUG
  } else if (comm_type == "can") {
    return; //no CAN for rover pro yet
  } else {
    return;
  }
}
statusData ProProtocolObject::translate_send_robot_status_request() {
  //TODO: DOUBLE CHECK
  // write_buffer[1] = (unsigned char)MOTOR_NEUTRAL;  // left motor
  // write_buffer[2] = (unsigned char)MOTOR_NEUTRAL;  // right motor
  // write_buffer[3] = (unsigned char)MOTOR_NEUTRAL;  // flipper
  write_buffer[4] = (unsigned char)10;  // Param 1: 10 to get data, 240 for low speed mode
  // write_buffer[5] = (unsigned char)param2;         // Param 2:
  // Calculate Checksum
  write_buffer[6] = (char)255 - (write_buffer[1] + write_buffer[2] +
  write_buffer[3] + write_buffer[4] + write_buffer[5]) % 255;
  comm_base->writetodevice(write_buffer);
  //Open a new thread for reading ?
  comm_base->readfromdevice();
}

void ProProtocolObject::translate_send_speed(double linearx, double angularz) {}

robotInfo ProProtocolObject::translate_send_robot_info_request() {
  // TODO:

}

void ProProtocolObject::unpack_robot_response() {
  // TODO: get robot response from comm manager
  // decode
}

bool ProProtocolObject::isConnected(){
  
}

}  // namespace RoverRobotics
