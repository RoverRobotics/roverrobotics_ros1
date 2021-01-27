
#include "comm_can.hpp"

namespace RoverRobotics {
CanManager::CanManager(const char* device) {}

CanManager::~CanManager() { close(serial_port); }

void CanManager::writetodevice(unsigned char* msg) {
  write(serial_port, msg, sizeof(msg));
}

char* CanManager::readfromdevice() {
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
  return read_buf;
}
}  // namespace RoverRobotics