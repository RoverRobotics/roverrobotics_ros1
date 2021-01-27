#pragma once
/*
This is the serial communication layer to the robot .
*/
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <cassert>
#include <ctime>
#include <iostream>
#include <string>

namespace RoverRobotics {
class CommBase;
}
class RoverRobotics::CommBase {
 public:
  CommBase() = default;
  virtual ~CommBase() = default;
  virtual void writetodevice(unsigned char*) = 0;
  virtual char* readfromdevice() = 0;
};