#include "protocol_pro2.hpp"

namespace RoverRobotics {

Pro2ProtocolObject::Pro2ProtocolObject(const char *device,
                                       std::string new_comm_type,
                                       bool closed_loop, PidGains pid,
                                       int motor_id[]) {
  comm_type = new_comm_type;
  closed_loop_ = closed_loop;
  estop_ = false;
  motors_speeds_[4] = {MOTOR_NEUTRAL};  // 0 all speed
  motors_id_[0] = motor_id[0];
  motors_id_[1] = motor_id[1];
  motors_id_[2] = motor_id[2];
  motors_id_[3] = motor_id[3];

  pid_ = pid;
  // !  std::vector<uint32_t> fast_data = {2, 4, 28, 30};
  // !  std::vector<uint32_t> slow_data = {10, 12, 20, 22, 38, 40, 64};
  motor1_control = OdomControl(closed_loop_, pid_, 250, 0, MOTOR_NEUTRAL);
  motor2_control = OdomControl(closed_loop_, pid_, 250, 0, MOTOR_NEUTRAL);
  register_comm_base(device);
  motor1_prev_t = std::chrono::steady_clock::now();
  motor2_prev_t = std::chrono::steady_clock::now();

  // data request threads
  // TODO
  //   writethread =
  //       std::thread([this, fast_data]() { this->sendCommand(50, fast_data);
  //       });

  //   writethread2 =
  //       std::thread([this, slow_data]() { this->sendCommand(50, slow_data);
  //       });
}
Pro2ProtocolObject::~Pro2ProtocolObject() {
  // Decontructor
}

void Pro2ProtocolObject::update_drivetrim(double value) { trimvalue = value; }

void Pro2ProtocolObject::translate_send_estop(bool estop) {
  writemutex.lock();
  estop_ = estop;
  writemutex.unlock();
}

statusData Pro2ProtocolObject::translate_send_robot_status_request() {
  return robotstatus_;
}

statusData Pro2ProtocolObject::translate_send_robot_info_request() {
  return robotstatus_;
}

void Pro2ProtocolObject::translate_send_speed(double *controlarray) {
  // prevent constant lock
  writemutex.lock();
  std::chrono::steady_clock::time_point motor1_prev_temp;
  std::chrono::steady_clock::time_point motor2_prev_temp;
  std::chrono::steady_clock::time_point motor3_prev_temp;
  std::chrono::steady_clock::time_point motor4_prev_temp;
  int firmware = robotstatus_.robot_firmware;
  double rpm1 = robotstatus_.motor1_rpm;
  double rpm2 = robotstatus_.motor2_rpm;
  double rpm3 = robotstatus_.motor3_rpm;
  double rpm4 = robotstatus_.motor4_rpm;

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
    double motor3_vel = linear_rate - 0.5 * diff_vel_commanded;
    double motor4_vel = linear_rate + 0.5 * diff_vel_commanded;
    double motor1_measured_vel =
        rpm1 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
    double motor2_measured_vel =
        rpm2 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
    double motor3_measured_vel =
        rpm3 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
    double motor4_measured_vel =
        rpm4 / MOTOR_RPM_TO_MPS_RATIO + MOTOR_RPM_TO_MPS_CFB;
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
    motors_speeds_[2] = motor3_control.run(
        motor3_vel, motor3_measured_vel,
        std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                              motor3_prev_temp)
                .count() /
            1000000.0,
        firmware);
    motors_speeds_[3] = motor4_control.run(
        motor4_vel, motor4_measured_vel,
        std::chrono::duration_cast<std::chrono::microseconds>(current_time -
                                                              motor4_prev_temp)
                .count() /
            1000000.0,
        firmware);

    std::cerr << "open loop motor command"
              << " left:" << (int)round(motor1_vel)
              << " right:" << (int)round(motor2_vel) << std::endl;
    std::cerr << "closed loop motor command"
              << " left:" << motors_speeds_[0] << " right:" << motors_speeds_[1]
              << std::endl;
  } else {
    motors_speeds_[0] = MOTOR_NEUTRAL;
    motors_speeds_[1] = MOTOR_NEUTRAL;
    motors_speeds_[2] = MOTOR_NEUTRAL;
    motors_speeds_[3] = MOTOR_NEUTRAL;
  }

  writemutex.unlock();
}

void Pro2ProtocolObject::unpack_comm_response(std::vector<uint32_t> msg) {}

bool Pro2ProtocolObject::isConnected() { return comm_base->isConnect(); }

void Pro2ProtocolObject::register_comm_base(const char *device) {
  std::cerr << "checking communication interface" << std::endl;
  if (comm_type == "serial") {
    std::cerr << "making serial connection" << std::endl;
    std::vector<uint32_t> setting_;
    setting_.push_back(4098);
    setting_.push_back(8);
    setting_.push_back(8);
    comm_base = std::make_unique<CommSerial>(
        device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); },
        setting_);
  } else if (comm_type == "can") {
    std::cerr << "making can connection" << std::endl;
    comm_base = std::make_unique<CommCan>(
        device, [this](std::vector<uint32_t> c) { unpack_comm_response(c); });
  }
}

void Pro2ProtocolObject::sendCommand(int sleeptime,
                                     std::vector<uint32_t> datalist) {
  while (true) {
    // Param 1: 10 to get data, 240 for low speed mode
    for (int x : datalist) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sleeptime));  // 20Hz
      if (comm_type == "serial") {
        // TODO
        
        // writemutex.lock();
        // write_buffer[0] = (unsigned char)253;
        // write_buffer[1] = (unsigned char)motors_speeds_[0];  // left motor
        // write_buffer[2] = (unsigned char)motors_speeds_[1];  // right motor
        // write_buffer[3] = (unsigned char)motors_speeds_[2];  // flipper
        // write_buffer[4] = (unsigned char)10;
        // write_buffer[5] = (unsigned char)x;  // Param 2:
        // // Calculate Checksum
        // write_buffer[6] =
        //     (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3]
        //     +
        //                  write_buffer[4] + write_buffer[5]) %
        //                     255;
        // comm_base->writetodevice(write_buffer);
        // // std::cerr << "To Robot: ";
        // // for (int i = 0; i < sizeof(write_buffer); i++) {
        // //   std::cerr << int(write_buffer[i]) << " ";
        // //   // std::cerr <<  std::dec <<int(write_buffer[i]) << " ";
        // // }
        // // std::cout << std::endl;
        // writemutex.unlock();
      } else if (comm_type == "can") {
        writemutex.lock();
        for (int i = 0; x < 4; x++) {
          motors_speeds_[i] = clip(motors_speeds_[i], -0.95, 0.95);
          // ROS_INFO("trimmed DUTY %f", Duty);
          int32_t v = static_cast<int32_t>(motors_speeds_[i] * 100000.0);
          frame.can_id = motors_id_[i] | 0x80000000U;
          frame.can_dlc = 4;
          frame.data[0] =
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
          frame.data[1] =
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
          frame.data[2] =
              static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
          frame.data[3] = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);
        }
        writemutex.unlock();
        // nbytes = write(s, &frame, sizeof(struct can_frame));
      } else {   //! How did you get here?
        return;  // TODO: Return error ?
      }
    }
  }
}
float Pro2ProtocolObject::clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}
}  // namespace RoverRobotics
