#include <stdio.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "geometry_msgs/Twist.h"
#include "protocol_base.hpp"
#include "protocol_pro.hpp"
#include "protocol_zero.hpp"
#include "robot_info.hpp"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "status_data.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"
namespace RoverRobotics {
class ROSWrapper;
}
class RoverRobotics::ROSWrapper {
 private:
  //
  std::unique_ptr<BaseProtocolObject> robot_;
  // Pub Sub
  ros::Subscriber speed_command_subscriber_;  // listen to cmd_vel inputs
  ros::Subscriber trim_command_subscriber_;   // listen to trim value broadcast
  ros::Subscriber estop_trigger_subscriber_;  // listen to estop trigger inputs
  ros::Subscriber estop_reset_subscriber_;    // listen to estop reset inputs

  ros::Subscriber robot_info_subscriber;  // listen to robot_info request
  ros::Publisher robot_info_publisher;    // publish robot_unique info

  ros::Subscriber
      robot_status_subscriber;  // listen to user togglable inputs i.e estop
  ros::Publisher robot_status_publisher_;  // publish robot state (battery,
                                           // estop_status, speed)

  // parameter variables
  std::string speed_topic_;
  std::string estop_trigger_topic_;
  std::string estop_reset_topic_;
  std::string robot_status_topic_;
  float robot_status_frequency;
  std::string robot_info_request_topic_;
  std::string robot_info_topic_;
  std::string robot_type_;
  std::string trim_topic_;
  bool estop_state;
  float trimvalue;
  std::string device_port_;
  std::string comm_type_;
  // Timer
  ros::Timer robot_status_timer_;

 public:
  ROSWrapper(ros::NodeHandle *nh);
  ~ROSWrapper();
  void publishRobotStatus(const ros::TimerEvent &event);
  void publishRobotInfo();
  void callbackSpeedCommand(const geometry_msgs::Twist &msg);
  void callbackTrim(const std_msgs::Float32::ConstPtr &msg);
  void callbackEstopTrigger(const std_msgs::Bool::ConstPtr &msg);
  void callbackInfo(const std_msgs::Bool::ConstPtr &msg);
  void callbackEstopReset(const std_msgs::Bool::ConstPtr &msg);
};

RoverRobotics::ROSWrapper::ROSWrapper(ros::NodeHandle *nh) {
  estop_state = false;
  // robot type check; if fail will stop this whole node.
  if (!ros::param::get("~device_port", device_port_)) {
    device_port_ = "/rover_zero";
  }
  if (!ros::param::get("~device_port", comm_type_)) {
    comm_type_ = "can";
  }
  if (!ros::param::get("~robot_type", robot_type_)) {
    ROS_FATAL("No Robot Type set. Shutting down ROS");
    ros::shutdown();
  } else if (robot_type_ == "Pro") {
    // robot_ = new ProProtocolObject(device_port_.c_str(), comm_type_);
    robot_ = std::unique_ptr<ProProtocolObject>(
        new ProProtocolObject(device_port_.c_str(), comm_type_));
  } else if (robot_type_ == "Zero") {
    robot_ = std::unique_ptr<ZeroProtocolObject>(
        new ZeroProtocolObject(device_port_.c_str(), comm_type_));
  } else {
    ROS_FATAL("Unknown Robot Type. Shutting down ROS");
    ros::shutdown();
  }

  // Check if launch files have parameters set; Otherwise use hardcoded values
  if (!ros::param::get("~trim_topic", trim_topic_)) {
    trim_topic_ = "/trim";
  }
  if (!ros::param::get("~speed_topic", speed_topic_)) {
    speed_topic_ = "/cmd_vel";
  }
  if (!ros::param::get("~estop_trigger_topic", estop_trigger_topic_)) {
    estop_trigger_topic_ = "/estop_trigger";
  }
  if (!ros::param::get("~estop_reset_topic", estop_reset_topic_)) {
    estop_reset_topic_ = "/estop_reset";
  }
  if (!ros::param::get("~status_topic", robot_status_topic_)) {
    robot_status_topic_ = "~/status";
  }
  if (!ros::param::get("~status_frequency", robot_status_frequency)) {
    robot_status_frequency = 1.00;
  }
  if (!ros::param::get("~info_request_topic", robot_info_request_topic_)) {
    robot_info_request_topic_ = "/robot_request_info";
  }
  if (!ros::param::get("~info_topic", robot_info_topic_)) {
    robot_info_topic_ = "~/robot_unique_info";
  }
  trim_command_subscriber_ =
      nh->subscribe(trim_topic_, 1, &ROSWrapper::callbackSpeedCommand, this);
  speed_command_subscriber_ =
      nh->subscribe(speed_topic_, 10, &ROSWrapper::callbackSpeedCommand, this);
  estop_trigger_subscriber_ = nh->subscribe(
      estop_trigger_topic_, 10, &ROSWrapper::callbackEstopTrigger, this);
  estop_reset_subscriber_ = nh->subscribe(
      estop_reset_topic_, 10, &ROSWrapper::callbackEstopReset, this);
  robot_info_subscriber =
      nh->subscribe(robot_info_request_topic_, 10, &ROSWrapper::callbackInfo,
                    this);  // listen to robot_info request
  robot_info_publisher = nh->advertise<std_msgs::Int32MultiArray>(
      robot_info_topic_, 1);  // publish robot_unique info
  // robot_status_subscriber = nh->subscribe(robot_status_topic_, 10,
  // &ROSWrapper::callbackInfo, this);  //listen to user togglable inputs i.e
  // estop
  robot_status_publisher_ =
      nh->advertise<diagnostic_msgs::DiagnosticStatus>(robot_status_topic_, 10);
  robot_status_timer_ =
      nh->createTimer(ros::Duration(1.0 / robot_status_frequency),
                      &ROSWrapper::publishRobotStatus, this);
  ROS_INFO("Subscribers and Publishers are running...");

}

void RoverRobotics::ROSWrapper::publishRobotStatus(
    const ros::TimerEvent &event) {
  if (!robot_->isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    robot_status_timer_.stop();
    ros::shutdown();
  }
  statusData data = robot_->translate_send_robot_status_request();
  std_msgs::Int32MultiArray robot_status;
  robot_status.data.clear();
  robot_status.data.push_back(data.time);
  robot_status.data.push_back(data.motor1_id);
  robot_status.data.push_back(data.motor2_id);
  robot_status.data.push_back(data.motor3_id);
  robot_status.data.push_back(data.motor4_id);
  robot_status.data.push_back(data.motor1_rpm);
  robot_status.data.push_back(data.motor2_rpm);
  robot_status.data.push_back(data.motor3_rpm);
  robot_status.data.push_back(data.motor4_rpm);
  robot_status.data.push_back(data.motor1_current);
  robot_status.data.push_back(data.motor2_current);
  robot_status.data.push_back(data.motor3_current);
  robot_status.data.push_back(data.motor4_current);
  robot_status.data.push_back(data.motor1_temp);
  robot_status.data.push_back(data.motor2_temp);
  robot_status.data.push_back(data.motor3_temp);
  robot_status.data.push_back(data.motor4_temp);
  robot_status.data.push_back(data.battery_voltage);
  robot_status.data.push_back(data.power);
  robot_status.data.push_back(data.charge_status);
  robot_status_publisher_.publish(robot_status);
  ROS_INFO("publishing some robot info");
}

void RoverRobotics::ROSWrapper::publishRobotInfo() {
  if (!robot_->isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    robot_status_timer_.stop();
    ros::shutdown();
    return;
  }
  robotInfo data = robot_->translate_send_robot_info_request();
  std_msgs::Int32MultiArray robot_info;
  robot_info.data.clear();
  robot_info.data.push_back(data.time);
  robot_info.data.push_back(data.guid);
  robot_info.data.push_back(data.firmware);

  robot_info_publisher.publish(robot_info);
  ROS_INFO("publishing some robot info");
}
// call everytime speed_topic_ get data
void RoverRobotics::ROSWrapper::callbackSpeedCommand(
    const geometry_msgs::Twist &msg) {
  if (!estop_state) {
    robot_->translate_send_speed(msg.linear.x, msg.angular.z);
    ROS_INFO("sent %f %f to the robot", msg.linear.x, msg.angular.z);
  }
}

void RoverRobotics::ROSWrapper::callbackInfo(
    const std_msgs::Bool::ConstPtr &msg) {
      if(msg->data){
        publishRobotInfo();
      }
    }

void RoverRobotics::ROSWrapper::callbackEstopTrigger(
    const std_msgs::Bool::ConstPtr &msg) {
  estop_state = true;
  robot_->translate_send_estop();
}

void RoverRobotics::ROSWrapper::callbackEstopReset(
    const std_msgs::Bool::ConstPtr &msg) {
  estop_state = false;
}

void RoverRobotics::ROSWrapper::callbackTrim(
    const std_msgs::Float32::ConstPtr &msg) {
  robot_->update_drivetrim(msg->data);
}

RoverRobotics::ROSWrapper::~ROSWrapper(){

}
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
