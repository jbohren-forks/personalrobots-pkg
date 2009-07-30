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
#include <fcamlib.h>
  
uint16_t checksum(uint16_t *data)
{
  uint16_t sum = 0;
  for (int i = 0; i < FLASH_PAGE_SIZE / 2; i++)
    sum += htons(data[i]);
 return htons(0xFFFF - sum);
}

int write_name(char *if_name, char *ip_address, int sn, char *name, char *new_ip)
{
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
    ROS_FATAL("Could not communicate with camera after configuring IP. Is ARP set? Is %s accessible from %s?", ip_address, if_name);
    return -1;
  }
  
  unsigned char namebuff[FLASH_PAGE_SIZE];
  IdentityFlashPage *id = (IdentityFlashPage *) &namebuff;

  if(fcamReliableFlashRead(camera, FLASH_NAME_PAGENO, (uint8_t *) namebuff, NULL) != 0)
  {
    ROS_FATAL("Flash read error. Aborting.");
    return -2;
  }
 
  uint16_t chk = checksum((uint16_t *) namebuff);
  if (chk)
  {
    ROS_ERROR("Previous camera name had bad checksum. Error: %04x", chk);
  }
  
  id->cam_name[sizeof(id->cam_name) - 1] = 0;
  printf("Previous camera name was: %s.\n", id->cam_name);
  uint8_t *oldip = (uint8_t *) &id->cam_addr;
  printf("Previous camera IP: %i.%i.%i.%i.\n", oldip[0], oldip[1], oldip[2], oldip[3]);

  if (strlen(name) > sizeof(id->cam_name) - 1)
  {
    ROS_FATAL("Name is too long, the maximum number of characters is %i.", sizeof(id->cam_name) - 1);
    return -2;
  }
  bzero(namebuff, FLASH_PAGE_SIZE);
  strncpy(id->cam_name, name, sizeof(id->cam_name) - 1);
  id->cam_name[sizeof(id->cam_name) - 1] = 0;
  struct in_addr cam_ip;
  if (!inet_aton(new_ip, &cam_ip))
  {
    ROS_FATAL("This is not a valid IP address: %s", new_ip);
    return -2;
  }
  id->cam_addr = cam_ip.s_addr;
  id->checksum = checksum((uint16_t *) namebuff);

  if (fcamReliableFlashWrite(camera, FLASH_NAME_PAGENO, (uint8_t *) namebuff, NULL) != 0)
  {    
    ROS_FATAL("Flash write error. The camera name is an undetermined state.");
    return -2;
  }
  
  ROS_INFO("Success! Restarting camera, should take about 10 seconds to come back up after this.");

  fcamReconfigureFPGA(camera);

  return 0;
}

int main(int argc, char **argv)
{
  if (argc != 6) {
    fprintf(stderr, "Usage: %s <Name> <SetIP> <Interface> <Current IP address> <Serial number>\n", argv[0]);
    return -1;
  }

  char* name = argv[1];
  char* new_ip = argv[2];
  char* if_name = argv[3];
  char* ip_address = argv[4];
  int sn = atoi(argv[5]);

  write_name(if_name, ip_address, sn, name, new_ip);
}
