#pragma once
#include "comm_base.hpp"

namespace RoverRobotics {
class CommSerial;
}
class RoverRobotics::CommSerial : public RoverRobotics::CommBase {
 public:
  CommSerial(const char* device);
  ~CommSerial();
  void writetodevice(unsigned char*);
  char* readfromdevice();
 private:
  char read_buf[256];  // size ?
  int serial_port;
};
