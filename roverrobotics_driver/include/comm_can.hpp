#pragma once
#include "comm_base.hpp"
namespace RoverRobotics {
class CommCan;
}
class RoverRobotics::CommCan: public RoverRobotics::CommBase {
 public:
  CommCan(const char* device,std::function<void(unsigned char *)>);
  ~CommCan();
  void writetodevice(unsigned char*);
  void readfromdevice(std::function<void(unsigned char *)>);
  void clearbuffer();
  bool isConnect();

 private:
  char read_buf[256];  // size ?
  int serial_port;
};
