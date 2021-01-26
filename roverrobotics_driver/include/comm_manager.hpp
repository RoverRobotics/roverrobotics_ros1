
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
class CommManager;
}
class RoverRobotics::CommManager {
 public:
  CommManager();
  virtual ~CommManager();
  virtual void writetodevice(const char*);
  virtual char* readfromdevice();

 private:
  std::unique_ptr<CommManager> comm_manager;
  //   char read_buf[256];  // size ?
  //   int serial_port;
};