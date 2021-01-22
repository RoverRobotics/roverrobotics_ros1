
/*
This is the serial communication layer to the robot .
*/
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <cassert>
namespace RoverRobotics {
class CommManager;
}
class RoverRobotics::CommManager {
   public:
    void *readThread(void *arg);
    void *writeThread(std::string msg);
    void connect(std::string);
    void disconnect();

    CommManager(const std::string &device, BaudRate baudRate);
    ~CommManager();

   private:
    int serial_device;
};