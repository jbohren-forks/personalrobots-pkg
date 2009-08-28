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
    fprintf(stderr, "Usage: %s <camera URL>\n", argv[0]);
    fprintf(stderr, "Sends the camera a PacketReconfigureFPGA packet.\n");
    fprintf(stderr, "Note: Currently reconfigure_cam and reset_cam have the same effect.\n");
    fprintf(stderr, "In future versions reset_cam may reset the camera without reconfiguring\n");
    fprintf(stderr, "the FPGA.\n");
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

  if ( fcamTriggerControl( &camera, TRIG_STATE_INTERNAL ) != 0) {
    fprintf(stderr, "Could not communicate with camera after configuring IP. Is ARP set? Is %s accessible from %s?\n", camera.ip_str, camera.ifName);
    return -1;
  }
 
  if ( fcamReconfigureFPGA( &camera ) != 0) {
    fprintf(stderr, "Error sending ReconfigureFPGA to camera.\n");
    return -1;
  }

  fprintf(stderr, "Success!\n");

  return 0;
}
