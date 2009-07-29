#include "fcamlib.h"
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
  fcamCamListInit(&camList);

  // Discover any connected cameras, wait for 0.5 second for replies
  if( fcamDiscover(if_name, &camList, NULL, SEC_TO_USEC(0.5)) == -1) {
    fprintf(stderr, "Discover error\n");
    return -1;
  }

  if (fcamCamListNumEntries(&camList) == 0) {
    fprintf(stderr, "No cameras found\n");
    return -1;
  }

  // Open camera with requested serial number
  int index = fcamCamListFind(&camList, sn);
  if (index == -1) {
    fprintf(stderr, "Couldn't find camera with S/N %i\n", sn);
    return -1;
  }
  IpCamList* camera = fcamCamListGetEntry(&camList, index);

  // Configure the camera with its IP address, wait up to 500ms for completion
  int retval = fcamConfigure(camera, ip_address, SEC_TO_USEC(0.5));
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
    int retries = 20;
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    if (fcamReliableFlashRead(camera, i, buffer, &retries) != 0) {
      fprintf(stderr, "Flash read error\n");
      return -1;
    }
    if (retries < 20)
      fprintf(stderr, "x");
    if (fwrite(buffer, FLASH_PAGE_SIZE, 1, stdout) != 1)
    {
      fprintf(stderr, "error: fwrite did not write one item. Image will be corrupt.\n");
    }
  }
  
  fprintf(stderr, "\n");

  return 0;
}
