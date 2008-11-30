#include <cstdio>
#include <stdexcept>
#include <netdb.h>
#include "stage.h"
#include <pthread.h>
#include "ros/time.h"

using std::string;
        
static const double STAGE_ENC_TO_DEG = 0.00453032;
static const double STAGE_DEG_TO_ENC = 1.0 / STAGE_ENC_TO_DEG;
static const long MIN_ENC_POS = (long)(STAGE_DEG_TO_ENC * 1.0);
static const long MAX_ENC_POS = (long)(STAGE_DEG_TO_ENC * 179.0);
static const long MAX_DUTY = 1023;

Stage::Stage()
: last_state(0), last_pos_deg(0),
  awaiting_state(false), awaiting_position(false)
{
  server_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (server_sock < 0)
    throw std::runtime_error("couldn't create server socket\n");

  pthread_t recv_thread_handle;
  pthread_create(&recv_thread_handle, NULL, s_recv_thread, this);
}

Stage::~Stage()
{
  close(server_sock);
  server_sock = 0;
  usleep(100000);
}

void Stage::set(string key, string value)
{
  printf("setting %s to %s\n", key.c_str(), value.c_str());
  if (key == "host")
    assign_host(value);
}

void Stage::home()
{
  gotoPosition(15, false);
}
  
bool Stage::gotoPosition(double deg, bool blocking, double max_wait)
{
  if (!blocking)
  {
    setPosEnc((long)(deg * STAGE_DEG_TO_ENC));
    return true;
  }
  else
  {
    long enc_target = (long)(deg * STAGE_DEG_TO_ENC);
    setPosEnc(enc_target);
    ros::Time t_start(ros::Time::now());
    while ((ros::Time::now() - t_start).to_double() < 
           t_start.to_double() + max_wait)
    {
      if (!getState(5))
        throw std::runtime_error("couldn't get state\n");
      if (stage_state == STAGE_IDLE  || 
          stage_state == STAGE_ERROR)
      {
        printf("got there\n");
        break;
      }
      usleep(100);
    }
    return ((ros::Time::now().to_double() < t_start.to_double() + max_wait) &&
            (stage_state != STAGE_ERROR));
  }
}

bool Stage::getState(double max_wait)
{
  awaiting_state = true;
  requestState();
  ros::Time t_start(ros::Time::now()), t_sent(ros::Time::now());
  while ((ros::Time::now() - t_start).to_double() < 
         t_start.to_double() + max_wait && awaiting_state)
  {
    if ((ros::Time::now() - t_sent).to_double() > 0.1)
    {
      requestState();
      t_sent = ros::Time::now();
    }
    usleep(100);
  }
  stage_state = (stage_state_t)last_state;
  if (awaiting_state)
    throw std::runtime_error("couldn't get stage state.\n");
  return true;
}
  
void Stage::setPosEnc(long pos)
{
  if (pos < MIN_ENC_POS)
    pos = (long)MIN_ENC_POS;
  else if (pos > MAX_ENC_POS)
    pos = (long)MAX_ENC_POS;
  uint8_t *p = (uint8_t *)&pos;
  uint8_t pkt[7];
  pkt[0] = CMD_MOVE;
  // todo: make this endian-neutral by bit-shifting here...
  pkt[1] = p[0];
  pkt[2] = p[1];
  pkt[3] = p[2];
  pkt[4] = p[3];
  pkt[5] = pkt[6] = 0x00;
  sendPacket(pkt, 7);
}

double Stage::getPosition(double max_wait)
{
  awaiting_position = true;
  requestPosition();
  ros::Time t_start(ros::Time::now()), t_sent(ros::Time::now());
  while ((ros::Time::now() - t_start).to_double() < 
         t_start.to_double() + max_wait && awaiting_position)
  {
    if ((ros::Time::now() - t_sent).to_double() > 0.1)
    {
      requestPosition();
      t_sent = ros::Time::now();
    }
  }
  if (awaiting_position)
    throw std::runtime_error("couldn't get stage position.\n");
  return last_pos_deg;
}
  
void Stage::assign_host(std::string host)
{
  hostent *hp;
  stage_addr.sin_family = AF_INET;
  stage_addr.sin_port = htons(STAGE_PORT);
  if (inet_addr(host.c_str()) == INADDR_NONE)
  {
    hp = gethostbyname(host.c_str());
    if (!hp)
    {
      printf("couldn't resolve host name [%s]\n", host.c_str());
      throw std::runtime_error("aborting due to unresolved host\n");
    }
    stage_addr.sin_addr.s_addr = *((unsigned long *)hp->h_addr);
    printf("resolved [%s] to [%s]\n", 
           host.c_str(), inet_ntoa(stage_addr.sin_addr));
  }
  else
    stage_addr.sin_addr.s_addr = inet_addr(host.c_str());
  printf("stage address: %s\n", inet_ntoa(stage_addr.sin_addr));
}

void *Stage::s_recv_thread(void *parent)
{
  ((Stage *)parent)->recv_thread();
  return NULL;
}

void Stage::recv_thread()
{
  char buf[100];
  while (server_sock)
  {
    int n = recv(server_sock, buf, 100, 0);
    if (n < 4)
      continue; // shouldn't happen
    switch (buf[0])
    {
      case CONTAINS_STATE:
        last_state = buf[1];
        awaiting_state = false;
        break;
      case CONTAINS_POSITION:
        last_pos_deg = *((long *)(buf+1)) * STAGE_ENC_TO_DEG;
        awaiting_position = false;
        break;
      case CONTAINS_ERROR:
        if (buf[1] == CONTAINS_ERROR)
          throw std::runtime_error("stage error\n");
        break;
    }
  }
}

void Stage::sendPacket(uint8_t *pkt, uint32_t len)
{
  placeChecksum(pkt, len-2);
  if (!sendto(server_sock, pkt, len, 0, (struct sockaddr *)&stage_addr,
              sizeof(stage_addr)))
  {
    printf("couldn't send packet\n");
    close(server_sock);
    server_sock = 0;
  }
}

void Stage::placeChecksum(uint8_t *pkt, unsigned len)
{
  int csum = calcChecksum(pkt, len);
  pkt[len] = (uint8_t)((csum >> 8) & 0xff);
  pkt[len+1] = (uint8_t)(csum & 0xff);
}

uint32_t Stage::calcChecksum(uint8_t *pkt, uint32_t len)
{
  uint32_t sum = 0;
  bool add_high_byte = true;
  for (uint32_t i = 0; i < len; i++)
  {
    sum += (pkt[i] << (add_high_byte ? 8 : 0)) ^ 
           (add_high_byte ? 0xff00 : 0x00ff);
    add_high_byte = !add_high_byte;
  }
  if (!add_high_byte)
    sum += 0xff;
  uint32_t checksum = ((sum >> 16) & 0xffff) + (sum & 0xffff);
  checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
  return checksum;
}

void Stage::requestState()
{
  uint8_t pkt[4];
  pkt[0] = pkt[1] = CMD_GET_STATE;
  pkt[2] = pkt[3] = 0x00;
  sendPacket(pkt, 4);
}

void Stage::requestPosition()
{
  uint8_t pkt[4];
  pkt[0] = pkt[1] = CMD_GET_POSITION;
  pkt[2] = pkt[3] = 0x00;
  sendPacket(pkt, 4);
}

void Stage::setDuty(long duty)
{
  if (duty < 0) duty = 0;
  else if (duty > MAX_DUTY) duty = MAX_DUTY;
  printf("set duty %ld\n", duty);
  uint8_t *d = (uint8_t *)&duty;
  uint8_t pkt[7];
  pkt[0] = CMD_SET_DUTY;
  pkt[1] = d[0];
  pkt[2] = d[1];
  pkt[3] = d[2];
  pkt[4] = d[3];
  pkt[5] = pkt[6] = 0x00;
  sendPacket(pkt, 7);
}

void Stage::laser(bool on)
{
  uint8_t pkt[4];
  if (on)
    pkt[0] = pkt[1] = CMD_SET_LASER_ON;
  else
    pkt[0] = pkt[1] = CMD_SET_LASER_OFF;
  pkt[2] = pkt[3] = 0x00;
  sendPacket(pkt, 4);
  usleep(10000);
  sendPacket(pkt, 4);
}

