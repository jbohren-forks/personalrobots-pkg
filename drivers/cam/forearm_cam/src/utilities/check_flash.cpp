#include "forearm_cam/fcamlib.h"
#include "forearm_cam/host_netutil.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <unistd.h>

int main(int argc, char** argv)
{
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <camera_url>\n", argv[0]);
    fprintf(stderr, "Writes a test sequence to unused portions of the flash to check for\n");
    fprintf(stderr, "proper operation. It is very improbable that you need this tool in\n");
    fprintf(stderr, "normal use of the camera.\n");
    return 0;
  }
  char* camera_url = argv[1];

  // Find the camera matching the URL
  IpCamList camera;
  const char *errmsg;
  int outval = fcamFindByUrl(camera_url, &camera, SEC_TO_USEC(0.1), &errmsg);
  if (outval)
  {
    fprintf(stderr, "Matching URL %s : %s\n", camera_url, errmsg);
    return -1;
  }

  // Configure the camera with its IP address, wait up to 500ms for completion
  int retval = fcamConfigure(&camera, camera.ip_str, SEC_TO_USEC(0.5));
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

  fprintf(stderr, "\n*************************************************************************\n");
  fprintf(stderr, "Warning, this will overwrite potentially important flash memory contents.\n");
  fprintf(stderr, "You have 5 seconds to press CTRL+C to abort this operation.\n");
  fprintf(stderr, "*************************************************************************\n");

  sleep(5);
  
  uint32_t incr = 282475249;
  uint32_t pattern = 0;
  for (int i = FLASH_MAX_PAGENO / 2; i <= FLASH_MAX_PAGENO - 10; i++)
  {
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    for (int j = 0; j < FLASH_SIZE_LONG; j++)
      buffer[j] = pattern += incr;

    if (fcamReliableFlashWrite(&camera, i, (uint8_t*) buffer, NULL) != 0) {
      fprintf(stderr, "\nFlash write error\n");
      return -1;
    }
  }
  printf("\n");
  
  int errcount = 0;
  pattern = 0;
  for (int i = FLASH_MAX_PAGENO / 2; i <= FLASH_MAX_PAGENO - 10; i++)
  {
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    for (int j = 0; j < FLASH_SIZE_LONG; j++)
      buffer[j] = pattern += incr;

    if (fcamReliableFlashRead(&camera, i, (uint8_t*) buffer2, NULL) != 0) {
      fprintf(stderr, "\nFlash read error\n");
      return -1;
    }
  
    if (memcmp(buffer, buffer2, FLASH_PAGE_SIZE))
    {
      errcount++;
      fprintf(stderr, "Mismatch in page %i\n.", i);
    }
  }

  fprintf(stderr, "\nCheck completed, %i errors.\n", errcount);

  return 0;
}
