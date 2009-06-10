#include "pr2lib.h"
#include "host_netutil.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <unistd.h>
  
int sensorread(IpCamList *camera, uint8_t reg)
{
  uint16_t val;

  if ( pr2SensorRead( camera, reg, &val ) != 0) {
    fprintf(stderr, "Could not get register.");
    return -1;
  }

  return val;
}

int sensorwrite(IpCamList *camera, uint8_t reg, uint16_t val)
{
  if ( pr2SensorWrite( camera, reg, val ) != 0) {
    fprintf(stderr, "Could not set register.");
    return -1;
  }
  return 0;
}

int main(int argc, char** argv)
{
  if (argc < 4 || argc > 7) {
    fprintf(stderr, "Usage: %s <interface> <IP address> <serial number> <register> [<value>]\nAssumes that the IP is already configured.\n", argv[0]);
    return 0;
  }
  char* if_name = argv[1];
  char* ip_address = argv[2];
  int sn = atoi(argv[3]);
  uint16_t val;
  uint8_t reg;
  if (argc >= 5)
    sscanf(argv[4], "%hhx", &reg);
  if (argc >= 6)
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

  if (argc >= 5 && !strcmp(argv[4], "readtst"))
  {
    int count = 0;
    if (argc >= 6)
      sscanf(argv[5], "%hhx", &reg);
    while (1)
    {
      int oldval = sensorread(camera, reg);
      fprintf(stderr, "count %i reg %02x read: %04x\n", count++, reg, oldval);
    }
  }
  else if (argc >= 5 && !strcmp(argv[4], "writetst"))
  {
    int count = 0;
    while (1)
    {
      if (argc >= 7)
      {
        sscanf(argv[5], "%hhx", &reg);
        sscanf(argv[6], "%hx", &val);
      }
      else
      {
        reg = 0;
        val = 0x1313;
      }
      sensorwrite(camera, reg, val);
      fprintf(stderr, "count %i reg %02x val %04x\n", count++, reg, val);
    }
  }
  else if (argc >= 5)
  {
    uint16_t oldval;
    oldval = sensorread(camera, reg);
    if (argc == 6)
    {
      sensorwrite(camera, reg, val);
      usleep(200000);
      int newval = sensorread(camera, reg);
      fprintf(stderr, "Reg %02x was: %04x set: %04x read: %04x\n", reg, oldval, val, newval);
    }
    else
      fprintf(stderr, "Reg %02x read: %04x\n", reg, oldval);
  }
  else
  {
    for (int i = 0; i < 256; i++)
    {
      if (i % 16 == 0)
        fprintf(stderr, "%02x:  ", i); 
      
      if (i % 16 == 8)
        fprintf(stderr, "- "); 
        
      fprintf(stderr, "%04x ", sensorread(camera, i));
      
      if ((i + 1) % 16 == 0)
        fprintf(stderr, "\n"); 
    }
  }

  return 0;
}
