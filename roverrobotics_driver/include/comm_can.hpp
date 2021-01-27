#pragma once
#include "comm_base.hpp"
namespace RoverRobotics {
class CanManager;
}
class RoverRobotics::CanManager: public RoverRobotics::CommBase {
 public:
  CanManager(const char* device);
  ~CanManager();
  void writetodevice(unsigned char*);
  char* readfromdevice();

 private:
  char read_buf[256];  // size ?
  int serial_port;
};
