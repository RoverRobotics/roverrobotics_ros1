#include <stdio.h>

#include <memory>

#include "geometry_msgs/Twist.h"
#include "librover/protocol_pro.hpp"
// #include "protocol_base.hpp"
// #include "protocol_pro.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "status_data.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "time.h"

namespace RoverRobotics {
class ROSWrapper;
}
class RoverRobotics::ROSWrapper {
 private:
  int motors_id_[4];
  //
  std::unique_ptr<BaseProtocolObject> robot_;
  // Pub Sub
  ros::Subscriber speed_command_subscriber_;  // listen to cmd_vel inputs
  ros::Subscriber trim_command_subscriber_;   // listen to trim value broadcast
  ros::Subscriber estop_trigger_subscriber_;  // listen to estop trigger inputs
  ros::Subscriber estop_reset_subscriber_;    // listen to estop reset inputs

  ros::Subscriber robot_info_subscriber;  // listen to robot_info request
  ros::Publisher robot_info_publisher;    // publish robot_unique info

  ros::Publisher robot_status_publisher_;  // publish robot state (battery,
                                           // estop_status, speed)
  ros::Publisher robot_odom_publisher_;    // publish odometry
  // parameter variables
  std::string speed_topic_;
  std::string estop_trigger_topic_;
  std::string estop_reset_topic_;
  std::string robot_status_topic_;
  float robot_status_frequency;
  float robot_odom_frequency;
  std::string robot_info_request_topic_;
  std::string robot_info_topic_;
  std::string robot_type_;
  std::string trim_topic_;
  float odom_angular_coef_;
  float odom_traction_factor_;

  float trimvalue;
  std::string device_port_;
  std::string comm_type_;
  // Timer
  ros::Timer robot_status_timer_;
  ros::Timer odom_publish_timer_;
  PidGains pidGains_ = {0, 0, 0};
  bool estop_state;
  bool closed_loop;

 public:
  ROSWrapper(ros::NodeHandle *nh);
  ~ROSWrapper();
  void publishRobotStatus(const ros::TimerEvent &event);
  void publishOdometry(const ros::TimerEvent &event);
  void publishRobotInfo();
  void callbackSpeedCommand(const geometry_msgs::Twist &msg);
  void callbackTrim(const std_msgs::Float32::ConstPtr &msg);
  void callbackEstopTrigger(const std_msgs::Bool::ConstPtr &msg);
  void callbackInfo(const std_msgs::Bool::ConstPtr &msg);
  void callbackEstopReset(const std_msgs::Bool::ConstPtr &msg);
};

RoverRobotics::ROSWrapper::ROSWrapper(ros::NodeHandle *nh) {
  motors_id_[4] = {0};
  estop_state = false;
  // IMPORTANT robot parameter
  // robot type check; if fail will stop this whole node.
  if (!ros::param::get("motor1_id", motors_id_[0])) {
  }
  if (!ros::param::get("motor2_id", motors_id_[1])) {
  }
  if (!ros::param::get("motor3_id", motors_id_[2])) {
  }
  if (!ros::param::get("motor4_id", motors_id_[3])) {
  }
  for (int i = 0; i < 4; i++) {
    std::cerr << "motor " << i + 1 << " id:" << motors_id_[i] << std::endl;
  }
  if (!ros::param::get("device_port", device_port_)) {
    ROS_FATAL("No 'device_port' set, Shutting down Driver Node");
    ros::shutdown();
  }
  if (!ros::param::get("comm_type", comm_type_)) {
    ROS_FATAL("No communication method set, Shutting down Driver Node");
    ros::shutdown();
  }
  if (!ros::param::get("closed_loop_control", closed_loop)) {
    ROS_INFO("no 'closed_loop_control' set; using the default value: 'false'");
    closed_loop = false;
  }
  //TODO update string to update as value change
  if (!ros::param::get("Kp", pidGains_.Kp)) {
    pidGains_.Kp = .5;
    ROS_INFO("no 'Kp' set; using the default value: '.5'");
  }
  if (!ros::param::get("Ki", pidGains_.Ki)) {
    pidGains_.Ki = 0;
    ROS_INFO("no 'Ki' set; using the default value: '.0'");
  }

  if (!ros::param::get("Kd", pidGains_.Kd)) {
    ROS_INFO("no 'Kd' set; using the default value: '0'");
    pidGains_.Kd = 0;
  }
  if (!ros::param::get("angular_coef", odom_angular_coef_)) {
    ROS_INFO("no 'angular_coef' set; using the default value: '0'");
    odom_angular_coef_ = 0;
  }
  if (!ros::param::get("traction_factor", odom_traction_factor_)) {
    ROS_INFO("no 'traction_factor' set; using the default value: '0'");
    odom_traction_factor_ = 0;
  }
  if (!ros::param::get("robot_type", robot_type_)) {
    ROS_FATAL(
        "No 'robot_type' found as a parameter. Shutting down Driver Node");
    ros::shutdown();
  } else if (robot_type_ == "pro") {
    robot_ = std::make_unique<ProProtocolObject>(
        device_port_.c_str(), comm_type_, closed_loop, pidGains_);
  } else {
    ROS_FATAL("Unknown Robot Type. Shutting down ROS");
    ros::shutdown();
  }
  std::cerr << "successfully setup robot interface" << std::endl;

  // Check if launch files have parameters set; Otherwise use hardcoded values
  if (!ros::param::get("trim_topic", trim_topic_)) {
    ROS_INFO("no 'trim_topic' set; using the default value: '/trim'");
    trim_topic_ = "/trim";
  }
  if (!ros::param::get("speed_topic", speed_topic_)) {
    ROS_INFO(
        "no 'speed_topic' set; using the default value: '/cmd_vel/managed'");
    speed_topic_ = "/cmd_vel/managed";
  }
  if (!ros::param::get("estop_trigger_topic", estop_trigger_topic_)) {
    ROS_INFO(
        "no 'estop_trigger_topic' set; using the default value: "
        "'/estop_trigger'");
    estop_trigger_topic_ = "/soft_estop_trigger";
  }
  if (!ros::param::get("estop_reset_topic", estop_reset_topic_)) {
    ROS_INFO(
        "no 'estop_reset_topic' set; using the default value: '/estop_reset'");
    estop_reset_topic_ = "/soft_estop_reset";
  }
  if (!ros::param::get("status_topic", robot_status_topic_)) {
    ROS_INFO("no 'status_topic' set; using the default value: '/robot_status'");
    robot_status_topic_ = "/robot_status";
  }
  if (!ros::param::get("status_frequency", robot_status_frequency)) {
    ROS_INFO("no 'status_frequency' set; using the default value: '1.00'");
    robot_status_frequency = 1.00;
  }
  if (!ros::param::get("odom_frequency", robot_odom_frequency)) {
    ROS_INFO("no 'odom_frequency' set; using the default value: '30.00'");
    robot_odom_frequency = 50.00;
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
  // if (!ros::param::get("estop_state", estop_state)) {
  //   ROS_INFO(
  //       "No 'estop_state' set; using the default value:
  //       '/robot_unique_info'");
  //   estop_state = false
  // }
  trim_command_subscriber_ =
      nh->subscribe(trim_topic_, 1, &ROSWrapper::callbackTrim, this);
  speed_command_subscriber_ =
      nh->subscribe(speed_topic_, 10, &ROSWrapper::callbackSpeedCommand, this);
  estop_trigger_subscriber_ = nh->subscribe(
      estop_trigger_topic_, 10, &ROSWrapper::callbackEstopTrigger, this);
  estop_reset_subscriber_ = nh->subscribe(
      estop_reset_topic_, 10, &ROSWrapper::callbackEstopReset, this);
  robot_info_subscriber =
      nh->subscribe(robot_info_request_topic_, 10, &ROSWrapper::callbackInfo,
                    this);  // listen to robot_info request
  robot_info_publisher = nh->advertise<std_msgs::Float32MultiArray>(
      robot_info_topic_, 1);  // publish robot_unique info

  robot_status_publisher_ =
      nh->advertise<std_msgs::Float32MultiArray>(robot_status_topic_, 10);
  robot_status_timer_ =
      nh->createTimer(ros::Duration(1.0 / robot_status_frequency),
                      &ROSWrapper::publishRobotStatus, this);
  robot_odom_publisher_ = nh->advertise<nav_msgs::Odometry>("odom", 1);
  odom_publish_timer_ =
      nh->createTimer(ros::Duration(1.0 / robot_odom_frequency),
                      &ROSWrapper::publishOdometry, this);
  ROS_INFO("Subscribers and Publishers are running...");
  ROS_INFO("ROBOT ESTOP STATE %d", estop_state);
}

void RoverRobotics::ROSWrapper::publishRobotStatus(
    const ros::TimerEvent &event) {
  if (!robot_->is_connected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
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

void RoverRobotics::ROSWrapper::publishOdometry(const ros::TimerEvent &event) {
  robotData data = robot_->status_request();
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = data.linear_vel;
  odom_msg.twist.twist.angular.z = data.angular_vel;
  robot_odom_publisher_.publish(odom_msg);
}

void RoverRobotics::ROSWrapper::publishRobotInfo() {
  if (!robot_->is_connected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
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
  robot_info_publisher.publish(robot_info);
}
// call everytime speed_topic_ get data
void RoverRobotics::ROSWrapper::callbackSpeedCommand(
    const geometry_msgs::Twist &msg) {
  double velocity_data[2];
  velocity_data[0] = msg.linear.x;
  velocity_data[1] = msg.angular.z;
  velocity_data[2] = msg.angular.y;
  robot_->set_robot_velocity(velocity_data);
}

void RoverRobotics::ROSWrapper::callbackInfo(
    const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    publishRobotInfo();
  }
}

void RoverRobotics::ROSWrapper::callbackEstopTrigger(
    const std_msgs::Bool::ConstPtr &msg) {
  estop_state = true;
  robot_->send_estop(estop_state);
}

void RoverRobotics::ROSWrapper::callbackEstopReset(
    const std_msgs::Bool::ConstPtr &msg) {
  estop_state = false;
  robot_->send_estop(estop_state);
}

void RoverRobotics::ROSWrapper::callbackTrim(
    const std_msgs::Float32::ConstPtr &msg) {
  robot_->update_drivetrim(msg->data);
}

RoverRobotics::ROSWrapper::~ROSWrapper() {}
int main(int argc, char **argv) {
  ros::init(argc, argv, "Rover Robotics ROS Driver");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);  // Prevent Callback bottleneck
  spinner.start();
  RoverRobotics::ROSWrapper robot(&nh);
  ROS_INFO("Robot driver is started");

  ros::waitForShutdown();
  // robot.~ROSWrapper();
  return 0;
}
