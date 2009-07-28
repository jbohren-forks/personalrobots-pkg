/*********************************************************************
 * * Software License Agreement (BSD License)
 * *
 * *  Copyright (c) 2008, Willow Garage, Inc.
 * *  All rights reserved.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *   * Redistributions of source code must retain the above copyright
 * *     notice, this list of conditions and the following disclaimer.
 * *   * Redistributions in binary form must reproduce the above
 * *     copyright notice, this list of conditions and the following
 * *     disclaimer in the documentation and/or other materials provided
 * *     with the distribution.
 * *   * Neither the name of the Willow Garage nor the names of its
 * *     contributors may be used to endorse or promote products derived
 * *     from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * *  POSSIBILITY OF SUCH DAMAGE.
 * *********************************************************************/

// Author: Blaise Gassend

#include <assert.h>
#include <iostream>
#include <fstream>
 
#include <string.h> // for memset(3)
#include <stdlib.h> // for atoi(3)

#include <ros/console.h>
#include <ros/time.h>

#include <ipcam_packet.h>
#include <host_netutil.h>
#include <pr2lib.h>
  
int write_name(char *if_name, char *ip_address, int sn, char *name)
{
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

  if ( pr2TriggerControl( camera, TRIG_STATE_INTERNAL ) != 0) {
    ROS_FATAL("Could not communicate with camera after configuring IP. Is ARP set? Is %s accessible from %s?", ip_address, if_name);
    return -1;
  }
  
  unsigned char chkbuff[FLASH_PAGE_SIZE];
  unsigned char namebuff[FLASH_PAGE_SIZE];

  bzero(namebuff, FLASH_PAGE_SIZE);
  strncpy((char *) namebuff, name, CAMERA_NAME_LEN);
  uint8_t checksum = 0;
  for (int i = 0; i < CAMERA_NAME_LEN; i++)
    checksum += namebuff[i];
  namebuff[CAMERA_NAME_LEN] = 255 - checksum;

  for (int i = 0; ; i++)
  {
      if (i > 20) {
        ROS_FATAL("Flash write error. The camera name is an undetermined state.");
        return -2;
      }
      
      if (pr2FlashWrite(camera, FLASH_NAME_PAGENO, (uint8_t *) namebuff) != 0)
      {
        printf("w");
        fflush(stdout);
        sleep(1);
        continue;
      }

      if (pr2FlashRead(camera, FLASH_NAME_PAGENO, (uint8_t *) chkbuff) != 0)
      {
        printf("r");
        fflush(stdout);
        sleep(1);
        continue;
      }
      
      if (memcmp(chkbuff, namebuff, FLASH_PAGE_SIZE)) 
      {
        printf("c");
        fflush(stdout);
        sleep(1);
        continue;
      }

      break;
  }
  printf("\n");
  
  ROS_INFO("Success!");

  return 0;
}

int main(int argc, char **argv)
{
  if (argc != 5) {
    fprintf(stderr, "Usage: %s <Name> <Interface> <IP address> <Serial number>\n", argv[0]);
    return -1;
  }

  char* name = argv[1];
  char* if_name = argv[2];
  char* ip_address = argv[3];
  int sn = atoi(argv[4]);

  write_name(if_name, ip_address, sn, name);
}
