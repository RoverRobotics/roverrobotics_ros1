#include "protocol_base.hpp"

namespace RoverRobotics {
class CanManager;
}
class RoverRobotics::CanManager : public RoverRobotics::CommManager {
 public:
  CanManager(std::string device);
  ~CanManager();
  void writetodevice(const char*);
  char* readfromdevice();
 private:
  char read_buf[256];  // size ?
  int serial_port;
};