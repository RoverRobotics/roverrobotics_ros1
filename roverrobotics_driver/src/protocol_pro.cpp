#include "protocol_pro.h"

namespace RoverRobotics {
ProProtocolObject::ProProtocolObject() {  //constructor
}
void ProProtocolObject::update_drivetrim(double value) {
    setTrim(value);
}
void translate_send_estop() {
    const MOTOR_NEUTRAL = 125;
    unsigned char write_buffer[SERIAL_OUT_PACKAGE_LENGTH];
    write_buffer[0] = SERIAL_START_BYTE;
    write_buffer[1] = (unsigned char)MOTOR_NEUTRAL;  // left motor
    write_buffer[2] = (unsigned char)MOTOR_NEUTRAL;  // right motor
    write_buffer[3] = (unsigned char)MOTOR_NEUTRAL;  // flipper
    // write_buffer[4] = (unsigned char)param1;         // Param 1: 10 to get data, 240 for low speed mode
    // write_buffer[5] = (unsigned char)param2;         // Param 2:
    // Calculate Checksum
    write_buffer[6] = (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] + write_buffer[4] + write_buffer[5]) % 255;
}
void translate_send_state_request(){
    //TODO: DOUBLE CHECK
    unsigned char write_buffer[SERIAL_OUT_PACKAGE_LENGTH];
    write_buffer[0] = SERIAL_START_BYTE;
    // write_buffer[1] = (unsigned char)MOTOR_NEUTRAL;  // left motor
    // write_buffer[2] = (unsigned char)MOTOR_NEUTRAL;  // right motor
    // write_buffer[3] = (unsigned char)MOTOR_NEUTRAL;  // flipper
    write_buffer[4] = (unsigned char)10;         // Param 1: 10 to get data, 240 for low speed mode
    // write_buffer[5] = (unsigned char)param2;         // Param 2:
    // Calculate Checksum
    write_buffer[6] = (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] + write_buffer[4] + write_buffer[5]) % 255;
}

void translate_send_speed(double linearx, double angularz){

}

void translate_send_robot_info_request(){
    //TODO:
}

void unpack_robot_response(){
    //TODO: get robot response from comm manager
    //decode
}

void register_state_response_cb(){
    
}

}  // namespace RoverRobotics