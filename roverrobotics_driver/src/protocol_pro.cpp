#include "protocol_pro.hpp"

namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char *device,
                                     std::string new_comm_type,
                                     bool closed_loop, PidGains pid) {
  comm_type = new_comm_type;
  closed_loop_ = closed_loop;
  estop_ = false;
  motors_speeds_[0] = 125;
  motors_speeds_[1] = 125;
  motors_speeds_[2] = 125;
  pid_ = pid;
  std::vector<uint32_t> fast_data = {2, 4, 28, 30};
  std::vector<uint32_t> slow_data = {10, 12, 20, 22, 38, 40, 64};
  motor1_control = OdomControl(closed_loop_, pid_, 250, 0, MOTOR_NEUTRAL);
  motor2_control = OdomControl(closed_loop_, pid_, 250, 0, MOTOR_NEUTRAL);
  register_comm_base(device);
  motor1_prev_t = std::chrono::steady_clock::now();
  motor2_prev_t = std::chrono::steady_clock::now();

  writethread =
      std::thread([this, fast_data]() { this->sendCommand(50, fast_data); });

  writethread2 =
      std::thread([this, slow_data]() { this->sendCommand(50, slow_data); });
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

void ProProtocolObject::translate_send_speed(double *controlarray) {
  // prevent constant lock
  writemutex.lock();
  std::chrono::steady_clock::time_point motor1_prev_temp;
  std::chrono::steady_clock::time_point motor2_prev_temp;
  int firmware = robotstatus_.robot_firmware;
  double rpm1 = robotstatus_.motor1_rpm;
  double rpm2 = robotstatus_.motor2_rpm;
  writemutex.unlock();

  writemutex.lock();
  if (!estop_) {
    double linear_rate = controlarray[0];
    double turn_rate = controlarray[1];
    double flipper_rate = controlarray[2];
    std::cerr << linear_rate << " " << turn_rate << " " << flipper_rate;
    // apply trim value

    if (turn_rate == 0) {
      if (linear_rate > 0) {
        turn_rate = trimvalue;
      } else if (linear_rate < 0) {
        turn_rate = -trimvalue;
      }
    }
    double diff_vel_commanded = turn_rate;
    double motor1_vel = linear_rate - 0.5 * diff_vel_commanded;
    double motor2_vel = linear_rate + 0.5 * diff_vel_commanded;
    double motor1_measured_vel =
        rpm1 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
    double motor2_measured_vel =
        rpm2 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;

    motors_speeds_[2] = (int)round(flipper_rate + 125) % 250;
    std::cerr << "commanded motor speed from ROS (m/s): "
              << "left:" << motor1_vel << " right:" << motor2_vel << std::endl;
    std::cerr << "measured motor speed (m/s)"
              << " left:" << motor1_measured_vel
              << " right:" << motor2_measured_vel << std::endl;
    std::chrono::steady_clock::time_point current_time =
        std::chrono::steady_clock::now();
    motors_speeds_[0] = motor1_control.run(
        motor1_vel, motor1_measured_vel,
        std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                              motor1_prev_temp)
                .count() /
            1000000.0,
        firmware);
    motors_speeds_[1] = motor2_control.run(
        motor2_vel, motor2_measured_vel,
        std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                              motor2_prev_temp)
                .count() /
            1000000.0,
        firmware);

    std::cerr << "open loop motor command"
              << " left:" << (int)round(motor1_vel * 50 + MOTOR_NEUTRAL)
              << " right:" << (int)round(motor2_vel * 50 + MOTOR_NEUTRAL)
              << std::endl;
    std::cerr << "closed loop motor command"
              << " left:" << motors_speeds_[0] << " right:" << motors_speeds_[1]
              << std::endl;
  } else {
    motors_speeds_[0] = MOTOR_NEUTRAL;
    motors_speeds_[1] = MOTOR_NEUTRAL;
    motors_speeds_[2] = MOTOR_NEUTRAL;
  }

  writemutex.unlock();
  // sendCommand(0, 0);
}
void ProProtocolObject::unpack_comm_response(std::vector<uint32_t> robotmsg) {
  static std::vector<uint32_t> msgqueue;
  writemutex.lock();
  std::cerr << "From Robot: ";
  for (auto data  : robotmsg) {
    std::cerr << data << " ";
  }
  std::cerr << std::endl;
  

  msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                  robotmsg.end());  // insert robotmsg to msg list
  std::cerr << "In Queue: ";
  for (auto a : msgqueue) {
    std::cerr << a << " ";
  }
  std::cerr << std::endl;
  std::cerr << "First Queue Value " << msgqueue[0] << std::endl;
  // reduce until finding the first startbit
  std::cerr << "finding start bit";
  // remove first value until first start bit is found
  // while (msgqueue[0] != startbit && msgqueue.size() > 5) {
  //   std::cerr << ".";
  //   // remove the first element until finding start bit
  //   std::vector<uint32_t> temp;
  //   for (int x = 1; x < msgqueue.size(); x++) {
  //     temp.push_back(msgqueue[x]);
  //   }
  //   msgqueue.clear();
  //   msgqueue.resize(0);
  //   msgqueue = temp;
  //   temp.clear();
  // }
  if (msgqueue[0] != startbit && msgqueue.size() > 5) {
    int startbyte_index = 0;
    while (msgqueue[startbyte_index] != startbit &&
           startbyte_index < msgqueue.size())
      startbyte_index++;
    if (startbyte_index > msgqueue.size()) {
      msgqueue.clear();
      return;
    } else {
      std::vector<uint32_t> temp;
      // msgqueue.at
      for (int x = startbyte_index; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }

      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }
  }
  if (msgqueue[0] == 253) {  // if valid start bit
    std::cerr << "start bit found!";
    unsigned char start_byte_read, data1, data2, dataNO;
    int checksum, read_checksum;
    start_byte_read = msgqueue[0];
    dataNO = msgqueue[1];
    data1 = msgqueue[2];
    data2 = msgqueue[3];
    checksum =
        (255 - int(msgqueue[1]) - int(msgqueue[2]) - int(msgqueue[3])) % 255;
    if (checksum == int(msgqueue[4])) {  // verify checksum
      // (data1 << 8) + data2;
      int b = (data1 << 8) + data2;
      switch (int(msgqueue[1])) {
        case 0:  // bat total current
        case 2:  // motor1_rpm;
          motor1_prev_t = std::chrono::steady_clock::now();
          robotstatus_.motor1_rpm = b;
          break;
        case 4:  // motor2_rpm;
          motor2_prev_t = std::chrono::steady_clock::now();
          // std::cerr << int(b) << std::endl;
          robotstatus_.motor2_rpm = b;
          break;
        case 6:  // motor 3 sensor 1
          robotstatus_.motor3_sensor1 = b;
          break;
        case 8:  // motor 3 sensor 2
          robotstatus_.motor3_sensor2 = b;
          break;
        case 10:  // motor 1 current
          robotstatus_.motor1_current = b;
          break;
        case 12:  // motor 2 current
          robotstatus_.motor2_current = b;
          break;
        case 14:  // motor 1 motor encoder count

        case 16:  // motor 2 motor encoder count
        case 18:  // motor fault
          robotstatus_.robot_fault_flag = b;
          break;
        case 20:  // motor 1 motor temp
          robotstatus_.motor1_temp = b;
          break;
        case 22:  // motor 2 motor temp
          robotstatus_.motor2_temp = b;
          break;
        case 24:  // voltage battery a

        case 26:  // voltage battery b
        case 28:  // motor 1 encoder interval
        case 30:  // motor 2 encoder interval
        case 32:  // motor 3 encoder interval
        case 34:  // battery A %
        case 36:  // battery B %
        case 38:  // battery state of charge
          robotstatus_.battery1_SOC = b;
          break;
        case 40:  // build NO
          robotstatus_.robot_firmware = b;
          break;
        case 42:  // battery A current
        case 44:  // battery B current
        case 46:  // motor 3 angle
          robotstatus_.motor3_angle = b;
          break;
        case 48:  // system fan speed
          robotstatus_.robot_fan_speed = b;
          break;
        case 50:  // speed mode
        case 52:  // battery status A from BMS
        case 54:  // battery status B from BMS
        case 56:  // battery A flag from BMS
          robotstatus_.battery1_fault_flag = b;
          break;
        case 58:  // battery B flag from BMS
          robotstatus_.battery2_fault_flag = b;
          break;
        case 60:  // battery temp A from BMS
          robotstatus_.battery1_temp = b;
          break;
        case 62:  // battery temp B from BMS
          robotstatus_.battery2_temp = b;
          break;
        case 64:  // battery A voltage from BMS
          robotstatus_.battery1_voltage = b;
          break;
        case 66:  // battery B voltage from BMS
          robotstatus_.battery2_voltage = b;
          break;
        case 68:  // battery A current from BMS
          robotstatus_.battery1_current = b;
          break;
        case 70:  // battery B current from BMS
          robotstatus_.battery2_current = b;
          break;
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
      robotstatus_.battery2_SOC = robotstatus_.battery1_SOC;
      std::vector<uint32_t> temp;
      for (int x = 5; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
      // msgqueue = temp;
    } else {
      std::cerr << "failed checksum" << std::endl;
      std::vector<uint32_t> temp;
      for (int x = 1; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
      // msgqueue = temp;
    }

  } else {  // ran out of byte doesn't have valid start bit
    std::cerr << "no start bit found!";
    // ran out of data; waiting for more
    // // std::cerr << "clearing buffer" << int(a[0]) << std::endl;
    // std::cerr << "Invalid Stream From Robot: " << (int)a[0];  // start
    // std::cerr << " " << (int)a[1];  // marker
    // std::cerr << " " << (int)a[2];  // data 1
    // std::cerr << " " << (int)a[3];  // data 2
    // std::cerr << " " << (int)a[4];  // checksum
    // if (checksum == a[4]){
    //   std::cerr << " checksum verified";
    // }
    // std::cerr << std::endl;
  }
  std::cerr << std::endl;
  writemutex.unlock();
}  // namespace RoverRobotics

bool ProProtocolObject::isConnected() {
  comm_base->isConnect();
  // TODO: TBD
}

void ProProtocolObject::register_comm_base(const char *device) {
  if (comm_type == "serial") {
    std::cerr << "making serial connection" << std::endl;
    std::vector<uint32_t> setting_;
    setting_.push_back(4097);
    setting_.push_back(7);
    setting_.push_back(5);
    comm_base = std::make_unique<CommSerial>(
        device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
        setting_);
  } else if (comm_type == "can") {
    std::cerr << "not available" << std::endl;
    // comm_base = std::make_unique<CommCan>(
    //     device, [this](unsigned char *c) { unpack_robot_response(c); });
    // std::cerr << "making can connection" << std::endl;
  }
}

void ProProtocolObject::sendCommand(int sleeptime,
                                    std::vector<uint32_t> datalist) {
  while (true) {
    // Param 1: 10 to get data, 240 for low speed mode
    for (int x : datalist) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sleeptime));  // 20Hz
      if (comm_type == "serial") {
        writemutex.lock();
        std::vector<uint32_t> write_buffer = {(unsigned char)253,
                                              (unsigned char)motors_speeds_[0],
                                              (unsigned char)motors_speeds_[1],
                                              (unsigned char)motors_speeds_[2],
                                              (unsigned char)10,
                                              (unsigned char)x};

        write_buffer.push_back((char)255 -
                               (motors_speeds_[0] + motors_speeds_[1] +
                                motors_speeds_[2] + 10 + x) %
                                   255);
        comm_base->writetodevice(write_buffer);
        writemutex.unlock();
      } else if (comm_type == "can") {
        return;  //* no CAN for rover pro
      } else {   //! How did you get here?
        return;  // TODO: Return error ?
      }
    }
  }
}

}  // namespace RoverRobotics
