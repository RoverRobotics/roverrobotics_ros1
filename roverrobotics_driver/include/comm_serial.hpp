#pragma once
#include "comm_base.hpp"
#include <thread>
#include <mutex>
namespace RoverRobotics {
class CommSerial;
}
class RoverRobotics::CommSerial : public RoverRobotics::CommBase {
 public:
  CommSerial(const char* device,std::function<void(unsigned char *)>);
  ~CommSerial();
  void writetodevice(unsigned char*);
  void readfromdevice(std::function<void(unsigned char *)>);
  void clearbuffer();
  bool isConnect();
 private:
  std::mutex writemutex;
  std::mutex readmutex;
  unsigned char read_buf[5];  // size ?
  int serial_port;
  std::thread readthread;
};
