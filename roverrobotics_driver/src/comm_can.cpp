
#include "comm_can.hpp"

namespace RoverRobotics {
CommCan::CommCan(const char *device,
                 std::function<void(std::vector<uint32_t>)> parsefunction) {
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    throw("Error while opening socket", -1);
  }
  strcpy(ifr.ifr_name, device);
  ioctl(s, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    throw("Error in socket bind", -2);
  }
  // start read thread
  readthread = std::thread(
      [this, parsefunction]() { this->readfromdevice(parsefunction); });
}

CommCan::~CommCan() { close(s); }

void CommCan::writetodevice(std::vector<uint32_t> msg) {
  writemutex.lock();
  if (msg.size() == 6) {
    // convert msg to frame
    frame.can_id = msg[0];
    frame.can_dlc = msg[1];
    frame.data[0] = msg[2];
    frame.data[1] = msg[3];
    frame.data[2] = msg[4];
    frame.data[3] = msg[5];
    write(s, &frame, sizeof(struct can_frame));
  }
  writemutex.unlock();
}

void CommCan::readfromdevice(
    std::function<void(std::vector<uint32_t>)> parsefunction) {
  std::cerr << "Read Thread started" << std::endl;
  while (true) {
    readmutex.lock();
    int num_bytes = read(s, &receive_frame, sizeof(receive_frame));
    std::vector<uint32_t> msg ;
    for (int x = 0 ; x < sizeof(receive_frame); x++){
      msg.push_back(receive_frame.can_dlc);
      msg.push_back(receive_frame.can_id);
      msg.push_back(receive_frame.data[0]);
      msg.push_back(receive_frame.data[1]);
      msg.push_back(receive_frame.data[2]);
      msg.push_back(receive_frame.data[3]);
      msg.push_back(receive_frame.data[4]);
    }
    parsefunction(msg);
    readmutex.unlock();
  }
  // return read_buf;
}

bool CommCan::isConnect() {
  if (s < 0) {
    return false;
  } else
    return true;
}
}  // namespace RoverRobotics