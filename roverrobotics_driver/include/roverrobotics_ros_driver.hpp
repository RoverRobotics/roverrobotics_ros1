#include <stdio.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include "geometry_msgs/Twist.h"
#include "librover/control.hpp"
#include "librover/protocol_mini.hpp"
#include "librover/protocol_zero_2.hpp"
#include "librover/protocol_pro.hpp"
#include "librover/protocol_pro_2.hpp"
#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/JoyFeedbackArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "time.h"

namespace RoverRobotics {

class RobotDriver {
 private:
  std::unique_ptr<BaseProtocolObject> robot_;
  // Pub Sub
  ros::Subscriber speed_command_subscriber_;  // listen to cmd_vel inputs
  ros::Subscriber mode_trigger_subscriber_;   // listen to change mode inputs
  ros::Subscriber trim_command_subscriber_;   // listen to trim value broadcast
  ros::Subscriber estop_trigger_subscriber_;  // listen to estop trigger inputs
  ros::Subscriber estop_reset_subscriber_;    // listen to estop reset inputs

  ros::Subscriber robot_info_subscriber_;  // listen to robot_info request
  ros::Publisher robot_info_publisher_;    // publish robot_unique info

  ros::Publisher robot_status_publisher_;  // publish robot state (battery,
  // estop_status, speed)
  ros::Publisher feedback_publisher_;
  ros::Publisher robot_odom_publisher_;  // publish odometry
  // parameter variables
  std::string speed_topic_;
  std::string estop_trigger_topic_;
  std::string mode_trigger_topic_;
  std::string estop_reset_topic_;
  std::string robot_status_topic_;
  float robot_status_frequency_;
  float robot_odom_frequency_;
  std::string robot_info_request_topic_;
  std::string robot_info_topic_;
  std::string robot_type_;
  std::string trim_topic_;
  std::string mode_name_;
  float odom_angular_coef_;
  float odom_traction_factor_;
  float trimvalue_;
  std::string device_port_;
  std::string comm_type_;
  // Timer
  ros::Timer robot_status_timer_;
  ros::Timer odom_publish_timer_;

  bool estop_state_ = false;
  Control::pid_gains pidGains_ = {0, 0, 0};
  Control::robot_motion_mode_t robot_mode_;
  Control::angular_scaling_params angular_scaling_params_ = {0, 0, 0, 0, 0};
  const float PID_P_DEFAULT_ = 0;
  const float PID_I_DEFAULT_ = 0;
  const float PID_D_DEFAULT_ = 0;
  const float PID_P_MAX_ = 1;
  const float PID_P_MIN_ = 0;
  const float PID_I_MAX_ = 1;
  const float PID_I_MIN_ = 0;
  const float PID_D_MAX_ = 1;
  const float PID_D_MIN_ = 0;
  const float ROBOT_STATUS_FREQUENCY_DEFAULT_ = 5;
  const float ROBOT_STATUS_FREQUENCY_MAX_ = 60;
  const float ROBOT_STATUS_FREQUENCY_MIN_ = 5;
  const float ROBOT_ODOM_FREQUENCY_DEFAULT_ = 30.00;
  const float ODOM_ANGULAR_COEF_DEFAULT_ = 0;
  const float ODOM_TRCTION_FACTOR_DEFAULT_ = 0;
  const float ANGULAR_SCALING_A_DEFAULT_ = 0;
  const float ANGULAR_SCALING_B_DEFAULT_ = 0;
  const float ANGULAR_SCALING_C_DEFAULT_ = 0;
  const float ANGULAR_SCALING_MIN_DEFAULT_ = 1;
  const float ANGULAR_SCALING_MAX_DEFAULT_ = 1;

 public:
  RobotDriver(ros::NodeHandle *nh);
  ~RobotDriver();
  void publishRobotStatus(const ros::TimerEvent &event);
  void publishOdometry(const ros::TimerEvent &event);
  void publishRobotInfo();
  void callbackModeTrigger(const std_msgs::Bool::ConstPtr &msg);
  void callbackSpeedCommand(const geometry_msgs::Twist &msg);
  void callbackTrim(const std_msgs::Float32::ConstPtr &msg);
  void callbackEstopTrigger(const std_msgs::Bool::ConstPtr &msg);
  void callbackInfo(const std_msgs::Bool::ConstPtr &msg);
  void callbackEstopReset(const std_msgs::Bool::ConstPtr &msg);
};
}  // namespace RoverRobotics
