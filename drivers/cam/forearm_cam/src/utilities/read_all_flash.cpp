#include "pr2lib.h"
#include "host_netutil.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc != 4) {
    fprintf(stderr, "Usage: %s <Interface> <IP address> <Serial number>\n", argv[0]);
    return 0;
  }
  char* if_name = argv[1];
  char* ip_address = argv[2];
  int sn = atoi(argv[3]);

  // Create a new IpCamList to hold the camera list
  IpCamList camList;
  pr2CamListInit(&camList);

  // Discover any connected cameras, wait for 0.5 second for replies
  if( pr2Discover(if_name, &camList, NULL, SEC_TO_USEC(0.5)) == -1) {
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

  // Configure the camera with its IP address, wait up to 500ms for completion
  int retval = pr2Configure(camera, ip_address, SEC_TO_USEC(0.5));
  if (retval != 0) {
    if (retval == ERR_CONFIG_ARPFAIL) {
      fprintf(stderr, "Unable to create ARP entry (are you root?), continuing anyway\n");
    } else {
      fprintf(stderr, "IP address configuration failed\n");
      return -1;
    }
  }

  uint8_t buffer[FLASH_PAGE_SIZE];

  for (int i = 0; i <= FLASH_MAX_PAGENO; i++)
  {
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    if (pr2FlashRead(camera, i, buffer) != 0) {
      fprintf(stderr, "Flash read error\n");
      return -1;
    }
    fwrite(buffer, FLASH_PAGE_SIZE, 1, stdout);
  }
  
  printf("\n");

  return 0;
}
