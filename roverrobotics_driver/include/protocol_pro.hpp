#include "protocol_base.hpp"

namespace RoverRobotics {
class ProProtocolObject;
}
class RoverRobotics::ProProtocolObject: public RoverRobotics::BaseProtocolObject {
   private:
    double trimvalue;
    std::unique_ptr<CommManager> comm_manager;
    std::string comm_type;
    // mutex comm_manager_mutex;
    void (*state_response_cb_function)();
   public:
    ProProtocolObject(const char*  device, std::string new_comm_type);
    ~ProProtocolObject();
    void update_drivetrim(double);
    void translate_send_estop();
    void translate_send_state_request();
    void translate_send_speed(double,double);
    void translate_send_robot_info_request();
    void handle_unsupported_ros_message();
    void unpack_robot_response();
    //void register_state_response_cb(boost::function<int(void)> _f);
    void register_comm_manager();


};
