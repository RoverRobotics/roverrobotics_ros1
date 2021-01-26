#include "protocol_base.hpp"

namespace RoverRobotics {
class SerialManager;
}
class RoverRobotics::SerialManager : public RoverRobotics::CommManager {
 public:
  SerialManager(const char*);
  ~SerialManager();
  void writetodevice(const char*);
  char* readfromdevice();
 private:
  char read_buf[256];  // size ?
  int serial_port;
};
