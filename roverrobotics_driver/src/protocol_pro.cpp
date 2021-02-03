#include "protocol_pro.hpp"

#include <chrono>

float MOTOR_RPM_TO_MPS_RATIO = 13749 / 1.26;
float MOTOR_RPM_TO_MPS_CFB = -0.07;
namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char* device,
                                     std::string new_comm_type,
                                     bool closed_loop, PidGains pid) {
  comm_type = new_comm_type;
  closed_loop_ = closed_loop;
  motors_speeds_[0] = 125;
  motors_speeds_[1] = 125;
  motors_speeds_[2] = 125;
  std::cerr << "Close Loop Control Status: " << closed_loop_ << std::endl;
  pid_ = pid;
  std::vector<int> fast_data = {2, 4, 28, 30};
  std::vector<int> slow_data = {10, 12, 20, 22, 38, 40, 64};
  motor1_control = OdomControl(closed_loop_, pid_, 250, 0);
  motor2_control = OdomControl(closed_loop_, pid_, 250, 0);
  motor1_control.start(closed_loop_, pid_, 250, 0);
  motor2_control.start(closed_loop_, pid_, 250, 0);
  register_comm_base(device);
  motor1_prev_t = std::chrono::steady_clock::now();
  motor2_prev_t = std::chrono::steady_clock::now();

  writethread =
      std::thread([this, fast_data]() { this->sendCommand(50, fast_data); });

  writethread2 =
      std::thread([this, slow_data]() { this->sendCommand(50, slow_data); });
}
ProProtocolObject::~ProProtocolObject() {
  // Decontructor
}

void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }

void ProProtocolObject::translate_send_estop(bool estop) {
  writemutex.lock();
  estop_ = estop;
  writemutex.unlock();
}

statusData ProProtocolObject::translate_send_robot_status_request() {
  return robotstatus_;
}

statusData ProProtocolObject::translate_send_robot_info_request() {
  return robotstatus_;
}

void ProProtocolObject::translate_send_speed(double* controlarray) {
  writemutex.lock();
  double linear_rate = controlarray[0];
  double turn_rate = controlarray[1];
  double flipper_rate = controlarray[2];
  // apply trim value
  std::cerr << "Control parameters: linear:" << controlarray[0]
            << " turn:" << controlarray[1] << " flipper:" << controlarray[2]
            << std::endl;
  if (turn_rate == 0) {
    if (linear_rate > 0) {
      turn_rate = trimvalue;
    } else if (linear_rate < 0) {
      turn_rate = -trimvalue;
    }
  }
  double diff_vel_commanded = turn_rate;
  int motor1_speed = (int)round((linear_rate - 0.5 * diff_vel_commanded));
  int motor2_speed = (int)round((linear_rate + 0.5 * diff_vel_commanded));

  motors_speeds_[2] = (int)round(flipper_rate + 125) % 250;
  std::cerr << "commanded motor speed: "
            << "left:" << motor1_speed << " right:" << motor2_speed
            << std::endl;
  std::chrono::steady_clock::time_point current_time =
      std::chrono::steady_clock::now();
  motors_speeds_[0] = motor1_control.run(
      estop_, closed_loop_, motor1_speed,
      robotstatus_.motor1_rpm / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB,
      std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                            motor1_prev_t)
              .count() /
          1000000.0,
      robotstatus_.robot_firmware);
  motors_speeds_[1] = motor2_control.run(
      estop_, closed_loop_, motor2_speed,
      robotstatus_.motor2_rpm / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB,
      std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                            motor2_prev_t)
              .count() /
          1000000.0,
      robotstatus_.robot_firmware);
  std::cerr << "filtered motor speed"
            << " left:" << motors_speeds_[0] << " right:" << motors_speeds_[1]
            << std::endl;

  writemutex.unlock();
  // sendCommand(0, 0);
}

void ProProtocolObject::unpack_robot_response(unsigned char* a) {
  unsigned char checksum, readchecksum;

  if (int(a[0]) != 0xfd) {  // invalid clear and move on
    comm_base->clearbuffer();
  } else if (a[0] == 0xfd) {  // if valid starting
    checksum = 255 - (a[0] + a[1] + a[2]) % 255;
    if (checksum == a[4]) {  // verify checksum
      unsigned char b = ((a[3]) | (a[2]));
      switch (a[1]) {
        case 0x00:  // bat total current
        case 0x02:  // motor1_rpm;
          motor1_prev_t = std::chrono::steady_clock::now();
          robotstatus_.motor1_rpm = b;
          break;
        case 0x04:  // motor2_rpm;
          motor2_prev_t = std::chrono::steady_clock::now();
          robotstatus_.motor2_rpm = b;
          break;
        case 0x06:  // motor 3 sensor 1
          robotstatus_.motor3_sensor1 = b;
          break;
        case 0x08:  // motor 3 sensor 2
          robotstatus_.motor3_sensor2 = b;
          break;
        case 0x10:  // motor 1 current
          robotstatus_.motor1_current = b;
          break;
        case 0x12:  // motor 2 current
          robotstatus_.motor2_current = b;
          break;
        case 0x14:  // motor 1 motor encoder count

        case 0x16:  // motor 2 motor encoder count
        case 0x18:  // motor fault
          robotstatus_.robot_fault_flag = b;
          break;
        case 0x20:  // motor 1 motor temp
          robotstatus_.motor1_temp = b;
          break;
        case 0x22:  // motor 2 motor temp
          robotstatus_.motor2_temp = b;
          break;
        case 0x24:  // voltage battery a

        case 0x26:  // voltage battery b
        case 0x28:  // motor 1 encoder interval
        case 0x30:  // motor 2 encoder interval
        case 0x32:  // motor 3 encoder interval
        case 0x34:  // battery A %
        case 0x36:  // battery B %
        case 0x38:  // battery state of charge
          robotstatus_.battery1_SOC = b;
          robotstatus_.battery2_SOC = b;
          break;
        case 0x40:  // build NO
          robotstatus_.robot_firmware = b;
          break;
        case 0x42:  // battery A current
        case 0x44:  // battery B current
        case 0x46:  // motor 3 angle
          robotstatus_.motor3_angle = b;
          break;
        case 0x48:  // system fan speed
          robotstatus_.robot_fan_speed = b;
          break;
        case 0x50:  // speed mode
        case 0x52:  // battery status A from BMS
        case 0x54:  // battery status B from BMS
        case 0x56:  // battery A flag from BMS
          robotstatus_.battery1_fault_flag;
          break;
        case 0x58:  // battery B flag from BMS
          robotstatus_.battery2_fault_flag;
          break;
        case 0x60:  // battery temp A from BMS
          robotstatus_.battery1_temp = b;
          break;
        case 0x62:  // battery temp B from BMS
          robotstatus_.battery2_temp = b;
          break;
        case 0x64:  // battery A voltage from BMS
          robotstatus_.battery1_voltage = b;
          break;
        case 0x66:  // battery B voltage from BMS
          robotstatus_.battery2_voltage = b;
          break;
        case 0x68:  // battery A current from BMS
          robotstatus_.battery1_current = b;
          break;
        case 0x70:  // battery B current from BMS
          robotstatus_.battery2_current = b;
          break;
      }
    }
    // THESE VALUES ARE NOT AVAILABLE ON ROVER PRO
    robotstatus_.motor1_id = 0;
    robotstatus_.motor1_mos_temp = 0;
    robotstatus_.motor2_id = 0;
    robotstatus_.motor2_mos_temp = 0;
    robotstatus_.motor3_id = 0;
    robotstatus_.motor3_rpm = 0;
    robotstatus_.motor3_current = 0;
    robotstatus_.motor3_temp = 0;
    robotstatus_.motor3_mos_temp = 0;
    robotstatus_.motor4_id = 0;
    robotstatus_.motor4_rpm = 0;
    robotstatus_.motor4_current = 0;
    robotstatus_.motor4_temp = 0;
    robotstatus_.motor4_mos_temp = 0;
    robotstatus_.robot_guid = 0;
    robotstatus_.robot_speed_limit = 0;
    // std::cerr << "From Robot: " << (int)a[0];  // start
    // std::cerr << " " << (int)a[1];  // marker
    // std::cerr << " " << (int)a[2];  // data 1
    // std::cerr << " " << (int)a[3];  // data 2
    // std::cerr << " " << (int)a[4];  // checksum
    // if (checksum == a[4]){
    //   std::cerr << " checksum verified";
    // }
    // std::cerr << std::endl;
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

void ProProtocolObject::sendCommand(int sleeptime, std::vector<int> datalist) {
  while (true) {
    // Param 1: 10 to get data, 240 for low speed mode
    for (int x : datalist) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sleeptime));  // 20Hz
      if (comm_type == "serial") {
        writemutex.lock();
        write_buffer[0] = (unsigned char)253;
        write_buffer[1] = (unsigned char)motors_speeds_[0];  // left motor
        write_buffer[2] = (unsigned char)motors_speeds_[1];  // right motor
        write_buffer[3] = (unsigned char)motors_speeds_[2];  // flipper
        write_buffer[4] = (unsigned char)10;
        write_buffer[5] = (unsigned char)x;  // Param 2:
        // Calculate Checksum
        write_buffer[6] =
            (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] +
                         write_buffer[4] + write_buffer[5]) %
                            255;
        comm_base->writetodevice(write_buffer);
        // std::cerr << "To Robot: ";
        // for (int i = 0; i < sizeof(write_buffer); i++) {
        //   std::cerr << int(write_buffer[i]) << " ";
        //   // std::cerr <<  std::dec <<int(write_buffer[i]) << " ";
        // }
        // std::cout << std::endl;
        writemutex.unlock();

      } else if (comm_type == "can") {
        return;  //* no CAN for rover pro yet
      } else {   //! How did you get here?
        return;  // TODO: Return error ?
      }
    }
  }
}

}  // namespace RoverRobotics
