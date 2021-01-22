
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
void CommManager::*readThread(void *arg) {
    clock_t
    std::cout << clock_t.clock() << std::endl;
}

void CommManager::*writeThread(std::string msg) {
   std::cout << clock_t.clock() << std::endl;
}
}
CommManager::CommManager(const std::string &device, BaudRate baudRate) {
    pthread_t write_Thread;
    int ret_write;
    pthread_t read_Thread;
    int ret_read;
    ret_write = pthread_create(&write_Thread, NULL, &writeThread(), NULL);
    ret_read = pthread_create(&read_Thread, NULL, &readThread(), NULL);
}
void connect(std::string port) {
    serial_port = open(port, O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
}
