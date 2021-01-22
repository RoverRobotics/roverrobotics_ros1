#include <boost/bind.hpp>

#include "comm_manager.hpp"
namespace RoverRobotics {
class BaseProtocolObject;
}
class RoverRobotics::BaseProtocolObject {
   public:
    BaseProtocolObject() {}
    ~BaseProtocolObject() {}
    virtual void update_drivetrim();
    virtual void translate_send_estop();
    virtual void translate_send_speed();
    virtual void translate_send_state_request();
    virtual void translate_send_robot_info_request();
    virtual void handle_unsupported_ros_message();
    virtual void unpack_robot_response();
    // virtual void register_state_response_cb(boost::function<int(void)> _f);
    virtual void register_comm_manager();

   private:
    double trimvalue;
    CommManager comm_manager;
    // mutex comm_manager_mutex;
    void (*state_response_cb_function)();
};
