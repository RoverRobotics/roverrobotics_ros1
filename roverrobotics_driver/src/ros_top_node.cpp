#include "protocol_base.h"
#include "protocol_pro.h"
#include "protocol_zero.h"
#include "ros/ros.h"

// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <termios.h>

// #include <cmath>
// #include <ctime>
// #include <fstream>
// #include <iostream>
// #include <string>
// #include <vector>

namespace RoverRobotics {
class ROSWrapper {
   private:
    //
    std::unique_ptr<BaseProtocolObject> robot_;
    //Pub Sub
    ros::Subscriber speed_command_subscriber_;
    ros::Publisher robot_status_publisher_;

    //parameter variables
    std::string speed_topic_;
    std::string motor_status_topic_;
    int robot_status_frequency;
    std::string robot_type_;
    //Timer
    ros::Timer motor_status_timer_;

   public:
    ROSWrapper(ros::NodeHandle* nh) {
        //robot type check; if fail will stop this whole node.
        if (!ros::param::get("~robot_type", robot_type_)) {
            ROS_FATAL("No Robot Type set. Shutting down ROS");
            ros::shutdown();
        } else if (robot_type == "Pro") {
            robot_ = new ProProtocolObject;
        } else if (robot_type == "Zero") {
            robot_ = new ZeroProtocolObject;
        } else {
            ROS_FATAL("Unknown Robot Type. Shutting down ROS");
            ros::shutdown();
        }

        // Check if launch files have parameters set; Otherwise use hardcoded values
        if (!ros::param::get("~speed_topic", speed_topic_)) {
            speed_topic_ = "/cmd_vel";
        }
        if (!ros::param::get("~motor_status_topic", speed_topic_)) {
            speed_topic_ = "/status";
        }
        if (!ros::param::get("~robot_status_frequency", robot_status_frequency)) {
            robot_status_frequency = 5.0;
        }
        

        speed_command_subscriber_ = nh->subscribe(speed_topic_, 10, &ROSWrapper::callbackSpeedCommand, this);
        robot_status_publisher_ = nh->advertise<diagnostic_msgs::DiagnosticStatus>(motor_status_topic_, 10);
        motor_status_timer_ = nh->createTimer(ros::Duration(1.0 / robot_status_frequency), &ROSWrapper::publishRobotStatus, this);
    }

    void publishRobotStatus(const ros::TimerEvent& event) {
        //get robot status from RobotObject
        robot_->translate_send_robot_info_request();
        //return value from protocol layer ?
        robot_status_publisher_.publish("some status");
    }
    //call everytime speed_topic_ get data
    void callbackSpeedCommand(const geometry_msgs::Twist& msg) {
        robot_->translate_send_speed(msg.linear.x, msg.angular.z);
    }
    bool callbackEStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        robot_->translate_send_estop();
        res.success = true;
        res.message = "E-stop service triggered";
        return true;
    }
};

}  // namespace RoverRobotics

int main(int argc, char** argv) {
    ros::init(argc, argv, "Rover Robotics ROS Driver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);  //Prevent Callback bottleneck
    spinner.start();
    RoverRobotics::ROSWrapper robot(&nh);
    ROS_INFO("Robot driver is started");

    ros::waitForShutdown();
    robot.stop();
    return 0;
}
