#include "protocol_base.hpp"

namespace RoverRobotics {
class ProProtocolObject;
}
class RoverRobotics::ProProtocolObject: public RoverRobotics::BaseProtocolObject {
   public:
    ProProtocolObject(){}
    ~ProProtocolObject(){}
    void update_drivetrim(double);
    void translate_send_estop();
    void translate_send_state_request();
    void translate_send_speed(double,double);
    void translate_send_robot_info_request();
    void handle_unsupported_ros_message();
    void unpack_robot_response();
    //void register_state_response_cb(boost::function<int(void)> _f);
    void register_comm_manager();

   private:
    double trimvalue;
    CommManager comm_manager;
    // mutex comm_manager_mutex;
    void (*state_response_cb_function)();
};
