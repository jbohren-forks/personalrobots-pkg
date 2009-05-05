/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author: Blaise Gassend

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "pr2lib.h"
#include "host_netutil.h"

int discover(const std::string &if_name)
{
  // Create a new IpCamList to hold the camera list
  IpCamList camList;
  pr2CamListInit(&camList);

  // Set anti-spoofing filter off on camera interface. Needed to prevent
  // the first reply from the camera from being filtered out.
  // @todo Should we be setting rp_filter to zero here? This may violate
  // the user's secutity preferences?
  std::string rp_str = "sysctl net.ipv4.conf."+if_name+".rp_filter|grep -q 0||sysctl -q -w net.ipv4.conf."+if_name+".rp_filter=0";
  int retval = system(rp_str.c_str());
  if (retval == -1 || !WIFEXITED(retval) || WEXITSTATUS(retval))
  {
    fprintf(stderr, "Unable to set rp_filter to 0 on interface. Camera discovery is likely to fail.\n");
  }

  // Discover any connected cameras, wait for 0.5 second for replies
  if( pr2Discover(if_name.c_str(), &camList, SEC_TO_USEC(0.5)) == -1) {
    fprintf(stderr, "Discover error.\n");
    return -1;
  }

  if (pr2CamListNumEntries(&camList) == 0) {
    printf("No cameras found\n");
    return 0;
  }

  for (int i = 0; i < pr2CamListNumEntries(&camList); i++)
  {
    IpCamList *camera = pr2CamListGetEntry(&camList, i);
    uint8_t *mac = camera->mac;
    uint8_t *ip = (uint8_t *) &camera->ip;
    char pcb_rev = 0x0A + (0x0000000F & ntohl(camera->hw_version));
    int hdl_rev = 0x00000FFF & (ntohl(camera->hw_version)>>4);
    printf("Found camera #%u with S/N #%u, MAC: %02x:%02x:%02x:%02x:%02x:%02x, IP: %i.%i.%i.%i, PCB rev %X : HDL rev %3X : FW rev %3X\n", 
        camera->serial, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ip[0], ip[1], ip[2], ip[3], 
        pcb_rev, hdl_rev, ntohl(camera->fw_version));
  }

  return 0;
}


int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: discover <interface>\n");
    return 1;
  }

  return discover(argv[1]);
} 
