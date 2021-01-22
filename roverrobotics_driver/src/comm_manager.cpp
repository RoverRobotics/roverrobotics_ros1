#include "comm_manager.hpp"

void RoverRobotics::CommManager::*readThread(void) {
    std::clock_t c_start = std::clock();
    printf("Time %d", c_start);
}
static void *readThread_helper(void *context) {
    return ((RoverRobotics::CommManager *)context)->readThread();
}
void RoverRobotics::CommManager::*writeThread(std::string msg) {
    std::clock_t c_start = std::clock();
    printf("Time %d", c_start);
}
static void *writeThread_helper(void *context) {
    return ((RoverRobotics::CommManager *)context)->writeThread("a");
}
RoverRobotics::CommManager::CommManager() {}
RoverRobotics::CommManager::CommManager(const char &device, float baudRate) {
    pthread_t write_Thread;
    int ret_write;
    pthread_t read_Thread;
    int ret_read;
    ret_write = pthread_create(&write_Thread, NULL, &RoverRobotics::CommManager::writeThread_helper, NULL);
    ret_read = pthread_create(&read_Thread, NULL, &RoverRobotics::CommManager::readThread_helper, NULL);
}
RoverRobotics::CommManager::~CommManager() {}

void RoverRobotics::CommManager::connect(char &device) {
    serial_device = open(device, O_RDWR);
    if (serial_device < 0) {
        //printf("Error %i from open: %s\n", errno, strerror(errno));
    }
}
