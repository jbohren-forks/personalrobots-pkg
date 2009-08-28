#include "wge100_camera/fcamlib.h"
#include "wge100_camera/host_netutil.h"
#include <string.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc != 2 || !strcmp(argv[1], "--help")) {
    fprintf(stderr, "Usage: %s <camera_url> > dump.bin\n", argv[0]);
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

  uint8_t buffer[FLASH_PAGE_SIZE];

  for (int i = 0; i <= FLASH_MAX_PAGENO; i++)
  {
    int retries = 20;
    if (i % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }
    if (fcamReliableFlashRead(&camera, i, buffer, &retries) != 0) {
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
