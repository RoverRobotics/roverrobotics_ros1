#include "protocol_base.h"
#include "protocol_pro.h"
#include "protocol_zero.h"
#include "robot_info.hpp"
#include "ros/ros.h"
#include "status_data.hpp"
namespace RoverRobotics {
class ROSWrapper {
   private:
    //
    std::unique_ptr<BaseProtocolObject> robot_;
    //Pub Sub
    ros::Subscriber speed_command_subscriber_;  //listen to cmd_vel inputs
    ros::Subscriber estop_trigger_subscriber_;  //listen to cmd_vel inputs
    ros::Subscriber estop_reset_subscriber_;    //listen to cmd_vel inputs

    ros::Subscriber robot_info_subscriber;  //listen to robot_info request
    ros::Publisher robot_info_publisher;    //publish robot_unique info

    ros::Subscriber robot_status_subscriber;  //listen to user togglable inputs i.e estop
    ros::Publisher robot_status_publisher_;   //publish robot state (battery, estop_status, speed)

    //parameter variables
    std::string speed_topic_;
    std::string estop_trigger_topic_;
    std::string estop_reset_topic_;
    std::string robot_status_topic_;
    float robot_status_frequency;
    std::string robot_info_request_topic_;
    std::string robot_info_topic_;
    std::string robot_type_;
    //Timer
    ros::Timer robot_status_timer_;

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

        speed_command_subscriber_ = nh->subscribe(speed_topic_, 10, &ROSWrapper::callbackSpeedCommand, this);
        estop_trigger_subscriber_ = nh->subscribe(estop_trigger_topic_, 10, &ROSWrapper::callbackEstopTrigger, this);
        estop_reset_subscriber_ = nh->subscribe(estop_reset_topic_, 10, &ROSWrapper::callbackEstopReset, this);
        robot_info_subscriber = nh->subscribe(robot_info_request_topic_, 10, &ROSWrapper::callbackInfo, this);    //listen to robot_info request
        robot_info_publisher = nh_priv_.advertise<std_msgs::Int32MultiArray>(robot_info_topic_, 1);               //publish robot_unique info
        // robot_status_subscriber = nh->subscribe(robot_status_topic_, 10, &ROSWrapper::callbackInfo, this);  //listen to user togglable inputs i.e estop
        robot_status_publisher_ = nh->advertise<diagnostic_msgs::DiagnosticStatus>(robot_status_topic_, 10);
        robot_status_timer_ = nh->createTimer(ros::Duration(1.0 / robot_status_frequency), &ROSWrapper::publishRobotStatus, this);
        ROS_INFO("Subscribers and Publishers are running...");
    }

    void publishRobotStatus(const ros::TimerEvent& event) {
        statusData data = robot_->translate_send_robot_status_request();
        std_msgs::Int32MultiArray robot_status;
        robot_status.data.clear();
        robot_status.data.push_back(data->time);
        robot_status.data.push_back(data->motor1_id);
        robot_status.data.push_back(data->motor2_id);

        robot_status_publisher_.publish(motor_speeds_msg);
        ROS_INFO("publishing some robot info")
    }

    void publishRobotInfo(const ros::TimerEvent& event) {
        robotInfo data = robot_->translate_send_robot_info_request();

        std_msgs::Int32MultiArray robot_info;
        robot_info.data.clear();
        robot_info.data.push_back(data->time);
        robot_info.data.push_back(data->motor1_id);
        robot_info.data.push_back(data->motor2_id);

        robot_info_publisher.publish(robot_info);
        ROS_INFO("publishing some robot info")
    }
    //call everytime speed_topic_ get data
    void callbackSpeedCommand(const geometry_msgs::Twist& msg) {
        robot_->translate_send_speed(msg.linear.x, msg.angular.z);
        ROS_INFO("sent %f %f to the robot", msg.linear.x, msg.angular.z)
    }
    void callbackEstopTrigger(const std_msgs::Bool::ConstPtr& msg) {
        robot_->translate_send_estop();
        res.success = true;
        res.message = "E-stop service triggered";
        return true;
    }
    void callbackEstopReset(const std_msgs::Bool::ConstPtr& msg)
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
