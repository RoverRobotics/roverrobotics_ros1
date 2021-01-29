#pragma once
#include "comm_base.hpp"
#include <thread>
#include <mutex>
namespace RoverRobotics {
class CommSerial;
}
class RoverRobotics::CommSerial : public RoverRobotics::CommBase {
 public:
  CommSerial(const char* device,std::function<void(char *)>);
  ~CommSerial();
  void writetodevice(unsigned char*);
  void readfromdevice(std::function<void(char *)>);
 private:
  std::mutex readmutex;
  char read_buf[5];  // size ?
  int serial_port;
  std::thread readthread;
};
