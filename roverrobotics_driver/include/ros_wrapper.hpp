#include <stdio.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include "geometry_msgs/Twist.h"
#include "librover/protocol_pro.hpp"
#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "time.h"

namespace RoverRobotics {

class ROSWrapper {
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
  float robot_status_frequency_max = 60;
  float robot_status_frequency_min = 5;
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
  float pid_p_max = 1;
  float pid_p_min = 0;
  float pid_i_max = 1;
  float pid_i_min = 0;
  float pid_d_max = 1;
  float pid_d_min = 0;
  bool estop_state = false;
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
}  // namespace RoverRobotics
