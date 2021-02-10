#include "protocol_pro.hpp"

namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char *device,
                                     std::string new_comm_type,
                                     bool closed_loop, PidGains pid) {
  comm_type = new_comm_type;
  closed_loop_ = closed_loop;
  estop_ = false;
  motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL;
  motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL;
  motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL;
  std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
                                     REG_MOTOR_FB_RPM_RIGHT, EncoderInterval_0,
                                     EncoderInterval_1};
  std::vector<uint32_t> slow_data = {
      REG_MOTOR_FB_CURRENT_LEFT, REG_MOTOR_FB_CURRENT_RIGHT,
      REG_MOTOR_TEMP_LEFT,       REG_MOTOR_TEMP_RIGHT,
      REG_MOTOR_CHARGER_STATE,   BuildNO,
      BATTERY_VOLTAGE_A};
  pid_ = pid;
  motor1_control =
      OdomControl(closed_loop_, pid_, MOTOR_MAX, MOTOR_MIN, MOTOR_NEUTRAL);
  motor2_control =
      OdomControl(closed_loop_, pid_, MOTOR_MAX, MOTOR_MIN, MOTOR_NEUTRAL);
  register_comm_base(device);
  motor1_prev_t = std::chrono::steady_clock::now();
  motor2_prev_t = std::chrono::steady_clock::now();

  // Create a New Thread with 20 mili seconds sleep timer
  writethread =
      std::thread([this, fast_data]() { this->sendCommand(20, fast_data); });
  // Create a new Thread with 50 mili seconds sleep timer
  writethread2 =
      std::thread([this, slow_data]() { this->sendCommand(50, slow_data); });
}

void ProProtocolObject::update_drivetrim(double value) { trimvalue = value; }

void ProProtocolObject::send_estop(bool estop) {
  writemutex.lock();
  estop_ = estop;
  writemutex.unlock();
}

statusData ProProtocolObject::status_request() { return robotstatus_; }

statusData ProProtocolObject::info_request() { return robotstatus_; }

void ProProtocolObject::send_speed(double *controlarray) {
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
    // !Applying some Skid-steer math
    double diff_vel_commanded = turn_rate;
    double motor1_vel = linear_rate - 0.5 * diff_vel_commanded;
    double motor2_vel = linear_rate + 0.5 * diff_vel_commanded;

    double motor1_measured_vel =
        rpm1 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
    double motor2_measured_vel =
        rpm2 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;

    motors_speeds_[FLIPPER_MOTOR] =
        (int)round(flipper_rate + MOTOR_NEUTRAL) % MOTOR_MAX;
    std::cerr << "commanded motor speed from ROS (m/s): "
              << "left:" << motor1_vel << " right:" << motor2_vel << std::endl;
    std::cerr << "measured motor speed (m/s)"
              << " left:" << motor1_measured_vel
              << " right:" << motor2_measured_vel << std::endl;
    std::chrono::steady_clock::time_point current_time =
        std::chrono::steady_clock::now();
    motors_speeds_[LEFT_MOTOR] = motor1_control.run(
        motor1_vel, motor1_measured_vel,
        std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                              motor1_prev_temp)
                .count() /
            1000000.0,
        firmware);
    motors_speeds_[RIGHT_MOTOR] = motor2_control.run(
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
    motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL;
    motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL;
    motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL;
  }

  writemutex.unlock();
}
void ProProtocolObject::unpack_comm_response(std::vector<uint32_t> robotmsg) {
  static std::vector<uint32_t> msgqueue;
  writemutex.lock();
  std::cerr << "Byte's' From Robot: ";
  for (auto data : robotmsg) {
    std::cerr << data << " ";
  }
  std::cerr << std::endl;

  msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                  robotmsg.end());  // insert robotmsg to msg list
  std::cerr << "Byte's' In Queue: ";
  for (auto a : msgqueue) {
    std::cerr << a << " ";
  }
  std::cerr << std::endl;
  std::cerr << "finding start byte";
  // ! Delete bytes until valid start byte is found
  if (msgqueue[0] != startbyte && msgqueue.size() > RECEIVE_MSG_LEN) {
    int startbyte_index = 0;
    // !Did not find valid start byte in buffer
    while (msgqueue[startbyte_index] != startbyte &&
           startbyte_index < msgqueue.size())
      startbyte_index++;
    if (startbyte_index > msgqueue.size()) {
      msgqueue.clear();
      return;
    } else {  // !Reconstruct the vector so that the start byte is at the 0
              // position
      std::vector<uint32_t> temp;
      for (int x = startbyte_index; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }
  }
  if (msgqueue[0] == startbyte) {  // if valid start byte
    std::cerr << "start byte found!";
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
        case REG_PWR_TOTAL_CURRENT:
        case REG_MOTOR_FB_RPM_LEFT:
          motor1_prev_t = std::chrono::steady_clock::now();
          robotstatus_.motor1_rpm = b;
          break;
        case REG_MOTOR_FB_RPM_RIGHT:  // motor2_rpm;
          motor2_prev_t = std::chrono::steady_clock::now();
          // std::cerr << int(b) << std::endl;
          robotstatus_.motor2_rpm = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT1:
          robotstatus_.motor3_sensor1 = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT2:
          robotstatus_.motor3_sensor2 = b;
          break;
        case REG_MOTOR_FB_CURRENT_LEFT:
          robotstatus_.motor1_current = b;
          break;
        case REG_MOTOR_FB_CURRENT_RIGHT:
          robotstatus_.motor2_current = b;
          break;
        case REG_MOTOR_ENCODER_COUNT_LEFT:

        case REG_MOTOR_ENCODER_COUNT_RIGHT:
        case REG_MOTOR_FAULT_FLAG_LEFT:
          robotstatus_.robot_fault_flag = b;
          break;
        case REG_MOTOR_TEMP_LEFT:
          robotstatus_.motor1_temp = b;
          break;
        case REG_MOTOR_TEMP_RIGHT:
          robotstatus_.motor2_temp = b;
          break;
        case REG_PWR_BAT_VOLTAGE_A:

        case REG_PWR_BAT_VOLTAGE_B:
        case EncoderInterval_0:
        case EncoderInterval_1:
        case EncoderInterval_2:
        case REG_ROBOT_REL_SOC_A:
        case REG_ROBOT_REL_SOC_B:
        case REG_MOTOR_CHARGER_STATE:
          robotstatus_.battery1_SOC = b;
          break;
        case BuildNO:
          robotstatus_.robot_firmware = b;
          break;
        case REG_PWR_A_CURRENT:
        case REG_PWR_B_CURRENT:
        case REG_MOTOR_FLIPPER_ANGLE:
          robotstatus_.motor3_angle = b;
          break;
        case to_computer_REG_MOTOR_SIDE_FAN_SPEED:
          robotstatus_.robot_fan_speed = b;
          break;
        case to_computer_REG_MOTOR_SLOW_SPEED:
        case BATTERY_STATUS_A:
        case BATTERY_STATUS_B:
        case BATTERY_MODE_A:
          robotstatus_.battery1_fault_flag = b;
          break;
        case BATTERY_MODE_B:
          robotstatus_.battery2_fault_flag = b;
          break;
        case BATTERY_TEMP_A:
          robotstatus_.battery1_temp = b;
          break;
        case BATTERY_TEMP_B:
          robotstatus_.battery2_temp = b;
          break;
        case BATTERY_VOLTAGE_A:
          robotstatus_.battery1_voltage = b;
          break;
        case BATTERY_VOLTAGE_B:
          robotstatus_.battery2_voltage = b;
          break;
        case BATTERY_CURRENT_A:
          robotstatus_.battery1_current = b;
          break;
        case BATTERY_CURRENT_B:
          robotstatus_.battery2_current = b;
          break;
      }
      // !Same battery system for both A and B on this robot
      robotstatus_.battery2_SOC = robotstatus_.battery1_SOC;
      // !THESE VALUES ARE NOT AVAILABLE ON ROVER PRO
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

      std::vector<uint32_t> temp;
      // !Remove processed msg from queue
      for (int x = RECEIVE_MSG_LEN; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    } else {  // !Found start byte but the msg contents were invalid, throw away
              // broken message
      std::cerr << "failed checksum" << std::endl;
      std::vector<uint32_t> temp;
      for (int x = 1; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }

  } else {
    // !ran out of data; waiting for more
    std::cerr << "no start byte found!";
  }
  std::cerr << std::endl;
  writemutex.unlock();
}

bool ProProtocolObject::isConnected() { comm_base->isConnect(); }

void ProProtocolObject::register_comm_base(const char *device) {
  if (comm_type == "serial") {
    std::cerr << "making serial connection" << std::endl;
    std::vector<uint32_t> setting_;
    setting_.push_back(baudrate);
    setting_.push_back(RECEIVE_MSG_LEN);
    comm_base = std::make_unique<CommSerial>(
        device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
        setting_);
  } else if (comm_type == "can") {
    std::cerr << "not available" << std::endl;
    throw(-1);
  }
}

void ProProtocolObject::sendCommand(int sleeptime,
                                    std::vector<uint32_t> datalist) {
  while (true) {
    for (int x : datalist) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
      if (comm_type == "serial") {
        writemutex.lock();
        std::vector<uint32_t> write_buffer = {
            (unsigned char)startbyte,
            (unsigned char)motors_speeds_[LEFT_MOTOR],
            (unsigned char)motors_speeds_[RIGHT_MOTOR],
            (unsigned char)motors_speeds_[FLIPPER_MOTOR],
            (unsigned char)requestbyte,
            (unsigned char)x};

        write_buffer.push_back(
            (char)255 -
            (motors_speeds_[LEFT_MOTOR] + motors_speeds_[RIGHT_MOTOR] +
             motors_speeds_[FLIPPER_MOTOR] + requestbyte + x) %
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
