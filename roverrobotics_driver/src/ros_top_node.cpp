#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "protocol_base.h"
#include "protocol_pro.h"
#include "protocol_zero.h"
#include "ros/ros.h"

int main(int argc, char* argv[]) {
    /*clearly a syntax error*/
    x = 5;
    //Test
    BaseProtocolObject* robot;
    robot = new ProProtocolObject;
}
