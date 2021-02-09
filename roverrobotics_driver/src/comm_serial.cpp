
#include "comm_serial.hpp"

namespace RoverRobotics {
CommSerial::CommSerial(const char *device,
                       std::function<void(std::vector<uint32_t>)> parsefunction) {
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

// void CommSerial::writetodevice(unsigned char *msg) {
//   writemutex.lock();  // Over Protective
//   write(serial_port, msg, sizeof(msg));
//   writemutex.unlock();
// }
void CommSerial::writetodevice(std::vector<uint32_t> msg) {
  writemutex.lock();
  write_buffer[0] = (unsigned char)msg[0];
  write_buffer[1] = (unsigned char)msg[1];  // left motor
  write_buffer[2] = (unsigned char)msg[2];  // right motor
  write_buffer[3] = (unsigned char)msg[3];  // flipper
  write_buffer[4] = (unsigned char)msg[4];
  write_buffer[5] = (unsigned char)msg[5];  // Param 2:
  // Calculate Checksum
  write_buffer[6] =
      (char)255 - (write_buffer[1] + write_buffer[2] + write_buffer[3] +
                   write_buffer[4] + write_buffer[5]) %
                      255;
  write(serial_port, write_buffer, sizeof(write_buffer));
  writemutex.unlock();
}

void CommSerial::readfromdevice(
    std::function<void(std::vector<uint32_t> )> parsefunction) {
  while (true) {
    readmutex.lock();
    unsigned char read_buf[5];  
    int num_bytes = read(serial_port, &read_buf, 5);
    std::vector<uint32_t> output; //TODO
    for (int x = 0; x < 5; x++){
      output.push_back(read_buf[x]);
    }
    parsefunction(output);
    readmutex.unlock();
  }
  //! THIS DO NOT SUPPORT DATA STREAM THAT IS BIGGER THAN 5 BYTE ATM
}

bool CommSerial::isConnect() { return (serial_port > 0); }

}  // namespace RoverRobotics