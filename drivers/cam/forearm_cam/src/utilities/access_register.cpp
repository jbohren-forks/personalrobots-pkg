#include "pr2lib.h"
#include "host_netutil.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <unistd.h>

int main(int argc, char** argv)
{
  if (argc != 5 && argc != 6) {
    fprintf(stderr, "Usage: %s <interface> <IP address> <serial number> <register> [<value>]\nAssumes that the IP is already configured.", argv[0]);
    return 0;
  }
  char* if_name = argv[1];
  char* ip_address = argv[2];
  int sn = atoi(argv[3]);
  uint16_t val;
  uint8_t reg;
  sscanf(argv[4], "%hhx", &reg);
  if (argc == 6)
    sscanf(argv[5], "%hx", &val);

  // Create a new IpCamList to hold the camera list
  IpCamList camList;
  pr2CamListInit(&camList);

  // Discover any connected cameras, wait for 0.5 second for replies
  if( pr2Discover(if_name, &camList, ip_address, SEC_TO_USEC(0.5)) == -1) {
    fprintf(stderr, "Discover error\n");
    return -1;
  }

  if (pr2CamListNumEntries(&camList) == 0) {
    fprintf(stderr, "No cameras found\n");
    return -1;
  }

  // Open camera with requested serial number
  int index = pr2CamListFind(&camList, sn);
  if (index == -1) {
    fprintf(stderr, "Couldn't find camera with S/N %i\n", sn);
    return -1;
  }
  IpCamList* camera = pr2CamListGetEntry(&camList, index);

  uint16_t oldval;
  if ( pr2SensorRead( camera, reg, &oldval ) != 0) {
    fprintf(stderr, "Could not get register. Is ARP set? Is %s accessible from %s?", ip_address, if_name);
    return -1;
  }

  if (argc == 6)
  {
    if ( pr2SensorWrite( camera, reg, val ) != 0) {
      fprintf(stderr, "Could not set register. Is ARP set? Is %s accessible from %s?", ip_address, if_name);
      return -1;
    }
  
    uint16_t newval;
    if ( pr2SensorRead( camera, reg, &newval ) != 0) {
      fprintf(stderr, "Could not get register. Is ARP set? Is %s accessible from %s?", ip_address, if_name);
      return -1;
    }

    fprintf(stderr, "Reg %02x was: %04x set: %04x read: %04x\n", reg, oldval, val, newval);
  }
  else
    fprintf(stderr, "Reg %02x read: %04x\n", reg, oldval);

  return 0;
}
