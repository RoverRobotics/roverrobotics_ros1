
#include "comm_can.hpp"

namespace RoverRobotics {
CommCan::CommCan(const char* device, std::function<void(unsigned char *)>) {}

CommCan::~CommCan() { close(serial_port); }

void CommCan::writetodevice(unsigned char* msg) {
  write(serial_port, msg, sizeof(msg));
}

void CommCan::readfromdevice(std::function<void(unsigned char *)> A) {
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
  // return read_buf;
}

void CommCan::clearbuffer(){}

bool CommCan::isConnect(){}
}  // namespace RoverRobotics