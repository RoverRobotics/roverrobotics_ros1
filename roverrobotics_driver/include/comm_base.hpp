#pragma once
/*
This is the serial communication layer to the robot .
*/
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "status_data.hpp"
#include "utils.hpp"
namespace RoverRobotics {
class CommBase;
}
class RoverRobotics::CommBase {
 public:
  CommBase() = default;
  virtual ~CommBase();
  // virtual void writetodevice(unsigned char*) = 0;
  virtual void writetodevice(std::vector<uint32_t>) = 0;
  // virtual void readfromdevice(std::function<void(unsigned char *)>) = 0;
  virtual void readfromdevice(std::function<void(std::vector<uint32_t>)>) = 0;
  virtual bool isConnect() = 0;
};