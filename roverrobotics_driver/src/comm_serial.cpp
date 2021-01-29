
#include "comm_serial.hpp"

namespace RoverRobotics {
CommSerial::CommSerial(const char* device) {
  // open serial port at specified port
  serial_port = open(device, 02);

  struct termios tty;
  if (tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno);
    return;
  }
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size tty.c_cflag
                           // |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 0;  // remove wait time

  // Set in/out baud rate to be 1152000
  cfsetispeed(&tty, B1152000);
  cfsetospeed(&tty, B1152000);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: \n", errno);
  }
}

CommSerial::~CommSerial() { close(serial_port); }

void CommSerial::writetodevice(unsigned char * msg) {
  msg[0] = 0x48; msg[1] = 0x49; msg[2] = 0x50;

  write(serial_port, msg, 3);
  for(int i=0; i<3; i++){
    //std::cout << std::dec << 3 << std::endl;
    std::cout << std::dec << msg[i] << " ";
  }
  std::cout << std::endl;
}

char* CommSerial::readfromdevice(){
      int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
      std::cout << read_buf[0] << std::endl;
      return read_buf;
}
}  // namespace RoverRobotics