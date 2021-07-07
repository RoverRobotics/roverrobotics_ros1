#include "roverrobotics_ros_driver.hpp"
namespace RoverRobotics {

RobotDriver::RobotDriver(ros::NodeHandle *nh) {
  ROS_INFO("Starting Rover Driver Node");
  // Robot Parameters
  estop_state_ = false;
  if (!ros::param::get("device_port", device_port_)) {
    ROS_FATAL("No 'device_port' set, Shutting down Driver Node");
    ros::shutdown();
  }
  if (!ros::param::get("comm_type", comm_type_)) {
    ROS_FATAL("No communication method set, Shutting down Driver Node");
    ros::shutdown();
  }
  mode_name_ = "OPEN_LOOP";
  if (!ros::param::get("robot_mode", mode_name_)) {
    ROS_INFO("no 'robot_mode' set; using the default value: 'OPEN_LOOP'");
  }

  if (mode_name_ == "TRACTION_CONTROL")
    robot_mode_ = Control::TRACTION_CONTROL;
  else if (mode_name_ == "INDEPENDENT_WHEEL")
    robot_mode_ = Control::INDEPENDENT_WHEEL;
  else
    robot_mode_ = Control::OPEN_LOOP;
  // PID Control
  if (robot_mode_ != Control::OPEN_LOOP) {
    ROS_WARN(
        "Closed Loop Control is ACTIVE. Please make sure your PID is properly "
        "tuned");
  }
  if (!ros::param::get("Kp", pidGains_.kp)) {
    pidGains_.kp = PID_P_DEFAULT_;
    ROS_INFO("no 'Kp' set; using the default value: %f", pidGains_.kp);
  }
  if (pidGains_.kp < PID_P_MIN_) {
    ROS_WARN("pidGains_.Kp is too low, changing to: %f", PID_P_MIN_);
    pidGains_.kp = PID_P_MIN_;
  } else if (pidGains_.kp > PID_P_MAX_) {
    ROS_WARN("pidGains_.Kp is too high, changing to: %f", PID_P_MAX_);
    pidGains_.kp = PID_P_MAX_;
  }
  if (!ros::param::get("Ki", pidGains_.ki)) {
    pidGains_.ki = PID_I_DEFAULT_;
    ROS_INFO("no 'Ki' set; using the default value: %f", PID_I_DEFAULT_);
  }
  if (pidGains_.ki < PID_I_MIN_) {
    ROS_WARN("pidGains_.Ki is too low, changing to: %f", PID_I_MIN_);
    pidGains_.ki = PID_I_MIN_;
  } else if (pidGains_.ki > PID_I_MAX_) {
    ROS_WARN("pidGains_.Ki is too high, changing to: %f", PID_I_MAX_);
    pidGains_.ki = PID_I_MAX_;
  }
  if (!ros::param::get("Kd", pidGains_.kd)) {
    ROS_INFO("no 'Kd' set; using the default value: %f", PID_D_DEFAULT_);
    pidGains_.kd = PID_D_DEFAULT_;
  }
  if (pidGains_.kd < PID_D_MIN_) {
    ROS_WARN("pidGains_.Kd is too low, changing to: %f", PID_D_MIN_);
    pidGains_.kd = PID_D_MIN_;
  } else if (pidGains_.kd > PID_D_MAX_) {
    ROS_WARN("pidGains_.Kd is too high, changing to: %f", PID_D_MAX_);
    pidGains_.kd = PID_D_MAX_;
  }
  //~PID Control
  if (!ros::param::get("angular_coef", odom_angular_coef_)) {
    ROS_INFO("no 'angular_coef' set; using the default value: %f",
             ODOM_ANGULAR_COEF_DEFAULT_);
    odom_angular_coef_ = ODOM_ANGULAR_COEF_DEFAULT_;
  }
  if (!ros::param::get("traction_factor", odom_traction_factor_)) {
    ROS_INFO("no 'traction_factor' set; using the default value: %f",
             ODOM_TRCTION_FACTOR_DEFAULT_);
    odom_traction_factor_ = ODOM_TRCTION_FACTOR_DEFAULT_;
  }
  if (!ros::param::get("angular_a_coef", angular_scaling_params_.a_coef)) {
    ROS_INFO("no 'angular_a_coef' set; using the default value: %f",
             ANGULAR_SCALING_A_DEFAULT_);
    angular_scaling_params_.a_coef = ANGULAR_SCALING_A_DEFAULT_;
  }
  if (!ros::param::get("angular_b_coef", angular_scaling_params_.b_coef)) {
    ROS_INFO("no 'angular_b_coef' set; using the default value: %f",
             ANGULAR_SCALING_B_DEFAULT_);
    angular_scaling_params_.b_coef = ANGULAR_SCALING_B_DEFAULT_;
  }
  if (!ros::param::get("angular_c_coef", angular_scaling_params_.c_coef)) {
    ROS_INFO("no 'angular_c_coef' set; using the default value: %f",
             ANGULAR_SCALING_C_DEFAULT_);
    angular_scaling_params_.c_coef = ANGULAR_SCALING_C_DEFAULT_;
  }
  if (!ros::param::get("angular_min_scale",
                       angular_scaling_params_.min_scale_val)) {
    ROS_INFO("no 'angular_min_scale' set; using the default value: %f",
             ANGULAR_SCALING_MIN_DEFAULT_);
    angular_scaling_params_.min_scale_val = ANGULAR_SCALING_MIN_DEFAULT_;
  }
  if (!ros::param::get("angular_max_scale",
                       angular_scaling_params_.max_scale_val)) {
    ROS_INFO("no 'angular_max_scale' set; using the default value: %f",
             ANGULAR_SCALING_MAX_DEFAULT_);
    angular_scaling_params_.max_scale_val = ANGULAR_SCALING_MAX_DEFAULT_;
  }
  if (!ros::param::get("robot_type", robot_type_)) {
    ROS_FATAL(
        "No 'robot_type' found as a parameter. Shutting down Driver Node");
    ros::shutdown();
  } else if (robot_type_ == "pro") {
    robot_ = std::make_unique<ProProtocolObject>(
        device_port_.c_str(), comm_type_, robot_mode_, pidGains_);
  } else if (robot_type_ == "pro2") {
    robot_ = std::make_unique<Pro2ProtocolObject>(
        device_port_.c_str(), comm_type_, robot_mode_, pidGains_,
        angular_scaling_params_);
  } else if (robot_type_ == "zero2") {
    robot_ = std::make_unique<Zero2ProtocolObject>(
        device_port_.c_str(), comm_type_, robot_mode_, pidGains_,
        angular_scaling_params_);
  } else if (robot_type_ == "mini") {
    robot_ = std::make_unique<MiniProtocolObject>(
        device_port_.c_str(), comm_type_, robot_mode_, pidGains_,
        angular_scaling_params_);
  } else {
    ROS_FATAL("Unknown Robot Type. Shutting down ROS");
    ros::shutdown();
    return;
  }

  // Check if launch files have parameters set; Otherwise use hardcoded values
  if (!ros::param::get("trim_topic", trim_topic_)) {
    ROS_INFO("no 'trim_topic' set; using the default value: '/trim_increment'");
    trim_topic_ = "/trim_increment";
  }
  if (!ros::param::get("speed_topic", speed_topic_)) {
    ROS_INFO(
        "no 'speed_topic' set; using the default value: '/cmd_vel/managed'");
    speed_topic_ = "/cmd_vel/managed";
  }
  if (!ros::param::get("estop_trigger_topic", estop_trigger_topic_)) {
    ROS_INFO(
        "no 'estop_trigger_topic' set; using the default value: "
        "'/soft_estop/trigger'");
    estop_trigger_topic_ = "/soft_estop/trigger";
  }
  if (!ros::param::get("estop_reset_topic", estop_reset_topic_)) {
    ROS_INFO(
        "no 'estop_reset_topic' set; using the default value: "
        "'/soft_estop/reset'");
    estop_reset_topic_ = "/soft_estop/reset";
  }
  if (!ros::param::get("status_topic", robot_status_topic_)) {
    ROS_INFO("no 'status_topic' set; using the default value: '/robot_status'");
    robot_status_topic_ = "/robot_status";
  }
  if (!ros::param::get("status_frequency", robot_status_frequency_)) {
    ROS_INFO("no 'status_frequency' set; using the default value: %f",
             ROBOT_STATUS_FREQUENCY_DEFAULT_);
    robot_status_frequency_ = ROBOT_STATUS_FREQUENCY_DEFAULT_;
  }
  if (robot_status_frequency_ < ROBOT_STATUS_FREQUENCY_MIN_) {
    ROS_WARN("status_frequency is too low, changing to default value: %f",
             ROBOT_STATUS_FREQUENCY_MIN_);
    robot_status_frequency_ = ROBOT_STATUS_FREQUENCY_MIN_;
  } else if (robot_status_frequency_ > ROBOT_STATUS_FREQUENCY_MAX_) {
    ROS_WARN("status_frequency is too high, changing to default value: %f",
             ROBOT_STATUS_FREQUENCY_MAX_);
    robot_status_frequency_ = ROBOT_STATUS_FREQUENCY_MAX_;
  }
  if (!ros::param::get("odom_frequency", robot_odom_frequency_)) {
    robot_odom_frequency_ = ROBOT_ODOM_FREQUENCY_DEFAULT_;
    ROS_INFO("no 'odom_frequency' set; using the default value: %f",
             robot_odom_frequency_);
  }
  if (!ros::param::get("info_request_topic", robot_info_request_topic_)) {
    ROS_INFO(
        "no 'info_request_topic' set; using the default value: "
        "'/robot_request_info'");
    robot_info_request_topic_ = "/robot_request_info";
  }
  if (!ros::param::get("info_topic", robot_info_topic_)) {
    ROS_INFO(
        "no 'info_topic' set; using the default value: '/robot_unique_info'");
    robot_info_topic_ = "/robot_unique_info";
  }
  if (!ros::param::get("mode_trigger_topic", mode_trigger_topic_)) {
    ROS_INFO(
        "no 'mode_trigger_topic' set; using the default value: '/mode_toggle'");
    mode_trigger_topic_ = "/mode_toggle";
  }

  trim_command_subscriber_ =
      nh->subscribe(trim_topic_, 1, &RobotDriver::callbackTrim, this);
  speed_command_subscriber_ =
      nh->subscribe(speed_topic_, 10, &RobotDriver::callbackSpeedCommand, this);
  mode_trigger_subscriber_ = nh->subscribe(
      mode_trigger_topic_, 10, &RobotDriver::callbackModeTrigger, this);
  estop_trigger_subscriber_ = nh->subscribe(
      estop_trigger_topic_, 10, &RobotDriver::callbackEstopTrigger, this);
  estop_reset_subscriber_ = nh->subscribe(
      estop_reset_topic_, 10, &RobotDriver::callbackEstopReset, this);
  robot_info_subscriber_ = nh->subscribe(robot_info_request_topic_, 10,
                                         &RobotDriver::callbackInfo, this);
  robot_info_publisher_ = nh->advertise<std_msgs::Float32MultiArray>(
      robot_info_topic_, 1);  // publish robot_unique info
  robot_status_publisher_ =
      nh->advertise<std_msgs::Float32MultiArray>(robot_status_topic_, 10);
  robot_status_timer_ =
      nh->createTimer(ros::Duration(1.0 / robot_status_frequency_),
                      &RobotDriver::publishRobotStatus, this);
  robot_odom_publisher_ = nh->advertise<nav_msgs::Odometry>("odom", 1);
  // feedback_publisher_ =
  //     nh->advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 1);
  odom_publish_timer_ =
      nh->createTimer(ros::Duration(1.0 / robot_odom_frequency_),
                      &RobotDriver::publishOdometry, this);
  ROS_INFO("Subscribers and Publishers are running...");
  ROS_INFO("ROBOT ESTOP STATE %d", estop_state_);
}

void RobotDriver::publishRobotStatus(const ros::TimerEvent &event) {
  if (!robot_->is_connected()) {
    ROS_FATAL(
        "Unexpectedly disconnected from serial port. Check connection to "
        "robot "
        "and reset Estop");
    robot_status_timer_.stop();
    ros::shutdown();
  }
  robotData data = robot_->status_request();
  std_msgs::Float32MultiArray robot_status;
  robot_status.data.clear();
  // Motor Infos
  robot_status.data.push_back(data.motor1_id);
  robot_status.data.push_back(data.motor1_rpm);
  robot_status.data.push_back(data.motor1_current);
  robot_status.data.push_back(data.motor1_temp);
  robot_status.data.push_back(data.motor1_mos_temp);
  robot_status.data.push_back(data.motor2_id);
  robot_status.data.push_back(data.motor2_rpm);
  robot_status.data.push_back(data.motor2_current);
  robot_status.data.push_back(data.motor2_temp);
  robot_status.data.push_back(data.motor2_mos_temp);
  robot_status.data.push_back(data.motor3_id);
  robot_status.data.push_back(data.motor3_rpm);
  robot_status.data.push_back(data.motor3_current);
  robot_status.data.push_back(data.motor3_temp);
  robot_status.data.push_back(data.motor3_mos_temp);
  robot_status.data.push_back(data.motor4_id);
  robot_status.data.push_back(data.motor4_rpm);
  robot_status.data.push_back(data.motor4_current);
  robot_status.data.push_back(data.motor4_temp);
  robot_status.data.push_back(data.motor4_mos_temp);
  // Battery Infos
  robot_status.data.push_back(data.battery1_voltage);
  robot_status.data.push_back(data.battery2_voltage);
  robot_status.data.push_back(data.battery1_temp);
  robot_status.data.push_back(data.battery2_temp);
  robot_status.data.push_back(data.battery1_current);
  robot_status.data.push_back(data.battery2_current);
  robot_status.data.push_back(data.battery1_SOC);
  robot_status.data.push_back(data.battery2_SOC);
  robot_status.data.push_back(data.battery1_fault_flag);
  robot_status.data.push_back(data.battery2_fault_flag);

  // Flipper Infos
  robot_status.data.push_back(data.motor3_angle);
  robot_status.data.push_back(data.motor3_sensor1);
  robot_status.data.push_back(data.motor3_sensor2);

  robot_status_publisher_.publish(robot_status);
  // ROS_INFO("publishing some robot state");
}

void RobotDriver::publishOdometry(const ros::TimerEvent &event) {
  robotData data = robot_->status_request();
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = data.linear_vel;
  odom_msg.twist.twist.angular.z = data.angular_vel;
  robot_odom_publisher_.publish(odom_msg);
}

void RobotDriver::publishRobotInfo() {
  if (!robot_->is_connected()) {
    ROS_FATAL(
        "Unexpectedly disconnected from serial port. Check connection to "
        "robot "
        "and reset Estop");
    robot_status_timer_.stop();
    ros::shutdown();
    return;
  }
  robotData data = robot_->info_request();
  std_msgs::Float32MultiArray robot_info;
  robot_info.data.clear();
  robot_info.data.push_back(data.robot_guid);
  robot_info.data.push_back(data.robot_firmware);
  robot_info.data.push_back(data.robot_speed_limit);
  robot_info.data.push_back(data.robot_fan_speed);
  robot_info.data.push_back(data.robot_fault_flag);
  robot_info_publisher_.publish(robot_info);
}
// call everytime speed_topic_ get data
void RobotDriver::callbackSpeedCommand(const geometry_msgs::Twist &msg) {
  double velocity_data[3];
  velocity_data[0] = msg.linear.x;
  velocity_data[1] = msg.angular.z;
  velocity_data[2] = msg.angular.y;
  robot_->set_robot_velocity(velocity_data);
}

void RobotDriver::callbackModeTrigger(const std_msgs::Bool::ConstPtr &msg) {
  int mode;
  sensor_msgs::JoyFeedbackArray a;
  if (msg->data) {
    mode = robot_->cycle_robot_mode();
    ROS_INFO("new mode %d", mode);
    switch (mode) {
      case Control::OPEN_LOOP:
        ROS_INFO("Robot Mode : Open Loop");
        break;
      case Control::TRACTION_CONTROL:
        ROS_INFO("Robot Mode : TRACTION_CONTROL");
        break;
      case Control::INDEPENDENT_WHEEL:
        ROS_INFO("Robot Mode : INDEPENDENT_WHEEL");

        break;
    }
  }

}

void RobotDriver::callbackInfo(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    publishRobotInfo();
  }
}

void RobotDriver::callbackEstopTrigger(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data == true) {
    estop_state_ = true;
    robot_->send_estop(estop_state_);
  }
}

void RobotDriver::callbackEstopReset(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data == true) {
    estop_state_ = false;
    robot_->send_estop(estop_state_);
  }
}

void RobotDriver::callbackTrim(const std_msgs::Float32::ConstPtr &msg) {
  if (msg->data != 0) {
    robot_->update_drivetrim(msg->data);
  }
}

RobotDriver::~RobotDriver() {}
}  // namespace RoverRobotics
int main(int argc, char **argv) {
  ros::init(argc, argv, "RoverRobotics_Driver_Wrapper_Node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);  // Prevent Callback bottleneck
  spinner.start();
  RoverRobotics::RobotDriver robot(&nh);
  ROS_INFO("Robot driver is started");

  ros::waitForShutdown();
  // robot.~RobotDriver();
  return 0;
}
