
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
    CommManager(const char &device, float baudRate);
    ~CommManager();
    static void *readThread_helper(void *context);
    static void *writeThread_helper(void *context);
    void *readThread(void *arg);
    void *writeThread(std::string msg);
    void connect(char &device);
    void disconnect();

   private:
    int serial_device;
};