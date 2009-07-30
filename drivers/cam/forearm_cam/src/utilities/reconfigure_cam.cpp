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

  if ( fcamTriggerControl( camera, TRIG_STATE_INTERNAL ) != 0) {
    fprintf(stderr, "Could not communicate with camera after configuring IP. Is ARP set? Is %s accessible from %s?", ip_address, if_name);
    return -1;
  }
 
  if ( fcamReconfigureFPGA( camera ) != 0) {
    fprintf(stderr, "Error sending ReconfigureFPGA to camera.");
    return -1;
  }

  fprintf(stderr, "Success!\n");

  return 0;
}