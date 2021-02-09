#pragma once
#include "comm_base.hpp"

namespace RoverRobotics {
class CommSerial;
}
class RoverRobotics::CommSerial : public RoverRobotics::CommBase {
 public:
  CommSerial(const char *device, std::function<void(std::vector<uint32_t>)>);
  ~CommSerial();
  // void writetodevice(unsigned char*);
  void writetodevice(std::vector<uint32_t> msg);
  // void readfromdevice(std::function<void(unsigned char *)>);
  void readfromdevice(std::function<void(std::vector<uint32_t>)>);

  void clearbuffer();
  bool isConnect();

 private:
  std::mutex writemutex;
  std::mutex readmutex;
  unsigned char write_buffer[7];
  unsigned char read_buf[5];  // size ?
  int serial_port;
  std::thread readthread;
};
