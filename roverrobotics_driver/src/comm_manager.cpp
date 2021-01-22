#include "comm_manager.hpp"

void RoverRobotics::CommManager::*readThread(void *arg) {
    clock_t t;
    std::cout << t.clock() << std::endl;
}

void RoverRobotics::CommManager::*writeThread(std::string msg) {
    clock_t t;
    std::cout << t.clock() << std::endl;
}
RoverRobotics::CommManager::CommManager(){}
RoverRobotics::CommManager::CommManager(const std::string &device, float baudRate) {
    pthread_t write_Thread;
    int ret_write;
    pthread_t read_Thread;
    int ret_read;
    ret_write = pthread_create(&write_Thread, NULL, &writeThread(), NULL);
    ret_read = pthread_create(&read_Thread, NULL, &readThread(), NULL);
}
RoverRobotics::CommManager::~CommManager() {}
void RoverRobotics::CommManager::connect(std::string port) {
    serial_port = open(port, O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
}
