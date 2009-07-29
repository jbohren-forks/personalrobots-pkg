#include "fcamlib.h"
#include "host_netutil.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <unistd.h>

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

#define FLASH_SIZE_LONG ((FLASH_PAGE_SIZE + 3) / 4)
  uint32_t buffer[FLASH_SIZE_LONG];
  uint32_t buffer2[FLASH_PAGE_SIZE];

  fprintf(stderr, "\n**************************************************************\n");
  fprintf(stderr, "Warning, this will overwrite the contents of the flash memory.\n");
  fprintf(stderr, "You have 5 seconds to press CTRL+C to abort this operation.\n");
  fprintf(stderr, "**************************************************************\n");

  sleep(5);
  
  uint32_t incr = 282475249;
  uint32_t pattern = 0;
  for (int i = FLASH_MAX_PAGENO / 2; i <= FLASH_MAX_PAGENO; i++)
  {
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    for (int j = 0; j < FLASH_SIZE_LONG; j++)
      buffer[j] = pattern += incr;

    if (fcamReliableFlashWrite(camera, i, (uint8_t*) buffer, NULL) != 0) {
      fprintf(stderr, "Flash write error\n");
      return -1;
    }
  }
  printf("\n");
  
  int errcount = 0;
  pattern = 0;
  for (int i = FLASH_MAX_PAGENO / 2; i <= FLASH_MAX_PAGENO; i++)
  {
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    for (int j = 0; j < FLASH_SIZE_LONG; j++)
      buffer[j] = pattern += incr;

    if (fcamReliableFlashRead(camera, i, (uint8_t*) buffer2, NULL) != 0) {
      fprintf(stderr, "Flash read error\n");
      return -1;
    }
  
    if (memcmp(buffer, buffer2, FLASH_PAGE_SIZE))
    {
      errcount++;
      fprintf(stderr, "Mismatch in page %i\n.", i);
    }
  }

  fprintf(stderr, "\nCheck completed, %i errors.", errcount);

  return 0;
}
