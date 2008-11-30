#ifndef STAGE_H
#define STAGE_H

// we are just using the rabbit-controlled stages right now; if we ever have
// more than one type of stage in use, i'll make this an abstract class

#include <string>
#include <arpa/inet.h>

class Stage
{
public:
  Stage();
  ~Stage();
  void set(std::string key, std::string value);
  void home();
  bool gotoPosition(double deg, bool blocking, double max_wait = 20.0);
  void setDuty(long duty);
  double getPosition(double max_wait);
  void laser(bool on);
private:
  sockaddr_in stage_addr;
  int server_sock;
  void assign_host(std::string host);
  static const unsigned short STAGE_PORT = 3211;
  static void *s_recv_thread(void *parent);
  void recv_thread();
  int8_t last_state;
  double last_pos_deg;
  bool awaiting_state, awaiting_position;
  static const uint8_t CMD_MOVE = 0x12;
  static const uint8_t CMD_SET_LASER_ON = 0x14;
  static const uint8_t CMD_SET_LASER_OFF = 0x15;
  static const uint8_t CMD_SET_DUTY = 0x16;
  static const uint8_t CMD_GET_STATE = 0x18;
  static const uint8_t CMD_GET_POSITION = 0x19;
  static const uint8_t CONTAINS_STATE = 0x20;
  static const uint8_t CONTAINS_POSITION = 0x21;
  static const uint8_t CONTAINS_ERROR = 0x22;
  static const uint8_t CONTAINS_DUTY = 0x23;
  static const uint8_t CONTAINS_LASER = 0x24;
  void sendPacket(uint8_t *pkt, uint32_t len);
  void placeChecksum(uint8_t *pkt, uint32_t len);
  uint32_t calcChecksum(uint8_t *pkt, uint32_t len);
  void setPosEnc(long pos);
  bool getState(double max_wait = 1.0);
  enum stage_state_t { STAGE_CALIBRATE = 0, STAGE_IDLE, STAGE_MOVE, 
                       STAGE_PATROL, STAGE_ERROR, STAGE_STEP_TO, 
                       STAGE_STEP_REST} stage_state;
  void requestState();
  void requestPosition();
};

#endif

