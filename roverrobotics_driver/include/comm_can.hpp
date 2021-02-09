#pragma once
#include "comm_base.hpp"

namespace RoverRobotics {
class CommCan;
}
class RoverRobotics::CommCan : public RoverRobotics::CommBase {
 public:
  CommCan(const char *device, std::function<void(std::vector<uint32_t>)>);
  ~CommCan();
  void writetodevice(std::vector<uint32_t> msg);
  void readfromdevice(std::function<void(std::vector<uint32_t>)>);
  bool isConnect();

 private:
  std::mutex writemutex;
  std::mutex readmutex;
  struct sockaddr_can addr;        // can address
  struct can_frame frame;          // Can Frame to robot
  struct ifreq ifr;                // can freq
  int s;                           // socket status
  struct can_frame receive_frame;  // Can Frame from robot

  std::thread readthread;
};
