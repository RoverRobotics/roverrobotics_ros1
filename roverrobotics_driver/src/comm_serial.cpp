
#include "comm_serial.hpp"

namespace RoverRobotics {
CommSerial::CommSerial(const char *device,
                       std::function<void(unsigned char *)> parsefunction) {
  // open serial port at specified port
  serial_port = open(device, 02);

  struct termios tty;
  if (tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: \n", errno);
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
  cfsetispeed(&tty, B57600);
  cfsetospeed(&tty, B57600);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: \n", errno);
  }
  readthread = std::thread(
      [this, parsefunction]() { this->readfromdevice(parsefunction); });
}

CommSerial::~CommSerial() { close(serial_port); }

void CommSerial::writetodevice(unsigned char *msg) {
  writemutex.lock();  // Over Protective
  write(serial_port, msg, sizeof(msg));
  writemutex.unlock();
}

void CommSerial::readfromdevice(
    std::function<void(unsigned char *)> parsefunction) {
  while (true) {
    readmutex.lock();
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
    parsefunction(read_buf);
    readmutex.unlock();
  }
  // return read_buf;
}

void CommSerial::clearbuffer() {
  read_buf[5] = {0};
}
bool CommSerial::isConnect(){
  return (serial_port > 0);
}

}  // namespace RoverRobotics