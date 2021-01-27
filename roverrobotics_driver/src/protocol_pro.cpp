#include "protocol_pro.hpp"

#include "can_manager.hpp"
#include "robot_info.hpp"
#include "serial_manager.hpp"
#include "status_data.hpp"
namespace RoverRobotics {
ProProtocolObject::ProProtocolObject(const char* device,
                                     std::string new_comm_type) {
  comm_type = new_comm_type;
  if (comm_type == "serial") {
    // comm_manager = new SerialManager(device); //bugged
    comm_manager = std::make_unique<SerialManager>(device);
  } else if (comm_type == "can") {
    comm_manager = std::make_unique<CanManager>(device);
  }
  comm_manager = new SerialManager(device);
}
ProProtocolObject::~ProProtocolObject() {}
void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }
void ProProtocolObject::translate_send_estop() {
  if (comm_type == "serial") {
    const int MOTOR_NEUTRAL = 125;
    unsigned char write_buffer[7];
    write_buffer[0] = 253;
    write_buffer[1] = (unsigned char)MOTOR_NEUTRAL;  // left motor
    write_buffer[2] = (unsigned char)MOTOR_NEUTRAL;  // right motor
    write_buffer[3] = (unsigned char)MOTOR_NEUTRAL; // flipper
    // write_buffer[4] = (unsigned char)param1;         // Param 1: 10 to get
    // data, 240 for low speed mode
    // write_buffer[5] = (unsigned char)param2;         // Param 2:
    // Calculate Checksum
    write_buffer[6] = (char)255 - (write_buffer[1] + write_buffer[2] +
    write_buffer[3] + write_buffer[4] + write_buffer[5]) % 255;
    comm_manager.writetodevice(write_buffer);
  } else if (comm_type == "can") {
  } else {
    return;
  }
}
void ProProtocolObject::translate_send_state_request() {
  // //TODO: DOUBLE CHECK
  // unsigned char write_buffer[SERIAL_OUT_PACKAGE_LENGTH];
  // write_buffer[0] = SERIAL_START_BYTE;
  // // write_buffer[1] = (unsigned char)MOTOR_NEUTRAL;  // left motor
  // // write_buffer[2] = (unsigned char)MOTOR_NEUTRAL;  // right motor
  // // write_buffer[3] = (unsigned char)MOTOR_NEUTRAL;  // flipper
  // write_buffer[4] = (unsigned char)10;  // Param 1: 10 to get data, 240 for
  // low speed mode
  // // write_buffer[5] = (unsigned char)param2;         // Param 2:
  // // Calculate Checksum
  // write_buffer[6] = (char)255 - (write_buffer[1] + write_buffer[2] +
  // write_buffer[3] + write_buffer[4] + write_buffer[5]) % 255;
}

void ProProtocolObject::translate_send_speed(double linearx, double angularz) {}

void ProProtocolObject::translate_send_robot_info_request() {
  // TODO:
}

void ProProtocolObject::unpack_robot_response() {
  // TODO: get robot response from comm manager
  // decode
}

// statusData RoverRobotics::ProProtocolObject::register_state_response_cb() {
// }
}  // namespace RoverRobotics
