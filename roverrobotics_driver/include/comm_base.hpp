#pragma once

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
  /* Accept a vector of unsigned int 32.
   * Convert vector of data into a buffer for then write to serial device*/
  virtual void writetodevice(std::vector<uint32_t> output) = 0;
  /* Accept a callback function.
   * Read from serial device then convert to an unsigned int 32 for the
   * callback*/
  virtual void readfromdevice(std::function<void(std::vector<uint32_t>)>) = 0;
  /* Check if the serial device is still connected.
   * Return boolean*/
  virtual bool isConnect() = 0;
};