
/*
This is the serial communication layer to the robot .
*/
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <cassert>
#include <string>

namespace RoverRobotics {
class CommManager;
}
class RoverRobotics::CommManager {
   public:
    CommManager();
    CommManager(const std::string &device, float baudRate);
    ~CommManager();
    void *readThread(void *arg);
    void *writeThread(std::string msg);
    void connect(std::string);
    void disconnect();

   private:
    int serial_device;
};