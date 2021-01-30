#include "protocol_pro.hpp"

namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char* device,
                                     std::string new_comm_type) {
  comm_type = new_comm_type;
  register_comm_base(device);
  translate_send_estop();
}

ProProtocolObject::~ProProtocolObject() {
  // Decontructor
}

void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }

void ProProtocolObject::translate_send_estop() {
  writemutex.lock();
  motors_speeds_[0] = MOTOR_NEUTRAL;
  motors_speeds_[1] = MOTOR_NEUTRAL;
  motors_speeds_[2] = MOTOR_NEUTRAL;
  writemutex.unlock();
}

statusData ProProtocolObject::translate_send_robot_status_request() {
  sendCommand(10, 4);
  robotstatus_.motor1_rpm = 808;
  // atof(read_buffer[0]); //convert char* to float from buffer
  // unpack_robot_response();
  return robotstatus_;
}

robotInfo ProProtocolObject::translate_send_robot_info_request() {
  // !:This robot have no special Info to request
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
  writemutex.lock();
  motors_speeds_[0] =
      (int)round((linear_rate - 0.5 * diff_vel_commanded) * 50 + MOTOR_NEUTRAL);
  motors_speeds_[1] =
      (int)round((linear_rate + 0.5 * diff_vel_commanded) * 50 + MOTOR_NEUTRAL);
  motors_speeds_[2] = (int)round(flipper_rate + 125) % 250;
  writemutex.unlock();
  sendCommand(0, 4);
}

void ProProtocolObject::handle_unsupported_ros_message() {
  // TODO: TBD
}

void ProProtocolObject::unpack_robot_response(unsigned char* a) {
  if (int(a[0]) != 0xfd) {  // invalid clear and move on
    comm_base->clearbuffer();
  } else if (a[0] == 0xfd) {  // if valid starting
    int b = ((a[3]) | (a[2]));
    switch (a[1]) {
      case 0x00:  // bat total current
      case 0x02:  // ?
      case 0x04:  // ?
      case 0x06:  // motor 3 sensor 1
      case 0x08:  // motor 3 sensor 2
      case 0x10:  // motor 1 current
        robotstatus_.motor1_current = b;
        break;
      case 0x12:  // motor 2 current
        robotstatus_.motor2_current = b;
        break;
      case 0x14:  // motor 1 motor encoder count

      case 0x16:  // motor 2 motor encoder count
      case 0x18:  // motor fault
      case 0x20:  // motor 1 motor temp
        robotstatus_.motor1_temp = b;
        break;
      case 0x22:  // motor 2 motor temp
        robotstatus_.motor2_temp = b;
        break;
      case 0x24:  // voltage battery a
      case 0x26:  // voltage battery b
      case 0x28:  // motor 1 encoder inverval
      case 0x30:  // motor 2 encoder interval
      case 0x32:  // motor 3 encoder interval
      case 0x34:  // battery A %
      case 0x36:  // battery B %
      case 0x38:  // battery state of charge
        robotstatus_.charge_status = b;
        break;
      case 0x40:  // build NO
        robotinfo_.guid = b;
        break;
      case 0x42:  // battery A current
      case 0x44:  // battery B current
      case 0x46:  // motor 3 angle
      case 0x48:  // system fan speed
      case 0x50:  // speed mode
      case 0x52:  // battery status A from BMS
      case 0x54:  // battery status B from BMS
      case 0x56:  // battery A flag from BMS
      case 0x58:  // battery B flag from BMS
      case 0x60:  // battery temp A from BMS
      case 0x62:  // battery temp B from BMS
      case 0x64:  // battery A voltage from BMS
        robotstatus_.battery_voltage = b;
        break;
      case 0x66:  // battery B voltage from BMS
      case 0x68:  // battery A current from BMS
      case 0x70:  // battery B current from BMS

        break;
    }
    // float motor1_rpm;
    // float motor2_rpm;
    // float motor3_rpm;
    // float motor1_current;
    // float motor2_current;
    // float motor3_current;
    // float battery_voltage;
    // float power;
    // std::cerr << "protocol received: " << (int)a[0] << std::endl;  // start
    // std::cerr << "protocol received: " << (int)a[1] << std::endl;  // marker
    // std::cerr << "protocol received: " << (int)a[2] << std::endl;  // data 1
    // std::cerr << "protocol received: " << (int)a[3] << std::endl;  // data 2
    // std::cerr << "protocol received: " << (int)a[4] << std::endl;  //
    // checksum
  }
}

bool ProProtocolObject::isConnected() {
  return true;
  // TODO: TBD
}

void ProProtocolObject::register_comm_base(const char* device) {
  if (comm_type == "serial") {
    comm_base = std::make_unique<CommSerial>(
        device, [this](unsigned char* c) { unpack_robot_response(c); });
    // comm_base = std::make_unique<CommSerial>(device,unpack_robot_response);
  } else if (comm_type == "can") {
    comm_base = std::make_unique<CommCan>(
        device, [this](unsigned char* c) { unpack_robot_response(c); });
    // comm_base = std::make_unique<CommCan>(device,unpack_robot_response);
  }
}

bool ProProtocolObject::sendCommand(int param1, int param2) {
  // Param 1: 10 to get data, 240 for low speed mode
  if (comm_type == "serial") {
    writemutex.lock();
    write_buffer[0] = (unsigned char)253;
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
    std::cerr << "To Robot: ";
    for (int i = 0; i < sizeof(write_buffer); i++) {
      std::cerr << int(write_buffer[i]) << " ";
      // std::cerr <<  std::dec <<int(write_buffer[i]) << " ";
    }
    std::cout << std::endl;
    writemutex.unlock();
  } else if (comm_type == "can") {
    return false;  //* no CAN for rover pro yet
  } else {         //! How did you get here?
    return false;  // TODO: Return error ?
  }
}

}  // namespace RoverRobotics
