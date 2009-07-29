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
#include <memory> // for std::auto_ptr
 
#include <string.h> // for memset(3)
#include <stdlib.h> // for atoi(3)

#include <ros/console.h>
#include <ros/time.h>

#include <ipcam_packet.h>
#include <host_netutil.h>
#include <fcamlib.h>
  
// We are assuming that the firmware will fit in the first half of the
// pages. This may not turn out to be true later...
#define FLASH_SIZE (FLASH_PAGE_SIZE * (FLASH_MAX_PAGENO + 1) / 2)
uint8_t firmware[FLASH_SIZE];
int  firmwarelen = 0;

int hexval(char c)
{
  assert(isxdigit(c));

  if (isdigit(c))
    return c - '0';
  if (isupper(c))
    return c - 'A' + 10;
  if (islower(c))
    return c - 'a' + 10;

  assert(false);

  return 0;
}

int read_mcs(std::string fname)
{      
  memset(firmware, 0xFF, FLASH_SIZE);
  firmwarelen = 0;
  std::ifstream file;
  file.open(fname.c_str());
  std::auto_ptr<std::vector<uint8_t> > data;
  int curseg = -1; // This will work for the forearm camera, but will break for larger address spaces.
  int linenum = 0;
  while (!file.eof())
  {
    linenum++;
    char line[100];
    file.getline(line, sizeof(line));
    unsigned char *curchar = (unsigned char *)line;
    
    if (*curchar++ != ':')
    {
      ROS_FATAL("Line %i did not start with a colon.", linenum);
      return -1;
    }
  
    // Declare bytes as int to avoid casting later.
    int bytes[100]; // Big enough because the line is only 100 characters. 
    int bytecount = 0;
    unsigned char checksum = 0;

    while (isxdigit(curchar[0]) && isxdigit(curchar[1]))
    {
      bytes[bytecount] = 16 * hexval(*curchar++); 
      bytes[bytecount] += hexval(*curchar++); 
      checksum += bytes[bytecount++];
    }

    while (*curchar && isspace(*curchar))
      curchar++;

    if (*curchar)
    {
      ROS_FATAL("Unexpected character 0x%02X at end of line %i.", 
          (int) *curchar, linenum);
      return -1;
    }

    /*for (int i = 0; i < bytecount; i++)
      printf("%02X ", bytes[i]);
    printf("\n");*/

    if (bytecount < 5)
    {
      ROS_FATAL("Line %i was too short (%i < 5).", linenum, bytecount); 
      return -1;
    }

    int len = bytes[0];

    if (len != bytecount - 5)
    {
      ROS_FATAL("Line %i contained %i bytes, but reported %i bytes.", 
          linenum, bytecount, bytes[0]);
      return -1;
    }
  
    int addr = (bytes[1] << 8) + bytes[2];

    switch (bytes[3])
    {
      case 0: // Data
        if (curseg + addr != firmwarelen)
        {
          ROS_FATAL("Non contiguous address (%08x != %08x), line %i",
              curseg + addr, firmwarelen, linenum);
          return -1;
        }
        if (curseg + addr + len >= FLASH_SIZE)
        {
          ROS_FATAL("Exceeded reserved flash size (note: upload_mcs code can be edited to allow larger values) at line %i.", linenum);
          return -1;
        }
        for (int i = 0; i < len; i++)
          firmware[firmwarelen++] = bytes[i + 4];
        break;

      case 1: // EOF
        while (true)
        {
          char c;
          file.get(c);
          if (file.eof())
            return 0;
          if (!isspace(c))
          {
            ROS_FATAL("EOF record on line %i was followed by character %02X.", 
                linenum, (int) c);
            return -1;
          }
        }

      case 4: // Extended address record
        if (len != 2)
        {
          ROS_FATAL("Extended record had wrong length (%i != 2) on line %i", 
              len, linenum);
          return -1;
        }
        curseg = ((bytes[4] << 8) + bytes[5]) << 16;
        break;

      default:
        ROS_FATAL("Unknown record type %i at line %i.", bytes[3], linenum);
        return -1;
    }
  }

  ROS_FATAL("Unexpected EOF after line %i.", linenum);
  return -1;
}

// For a serial flash, the bits in each byte need to be swapped around.
void bitswap()
{
  // Precompute bit-swap table
  uint8_t swapped[256];

  for (int i = 0; i < 256; i++)
  {
    uint8_t shift = i;
    swapped[i] = 0;
    for (int j = 0; j < 8; j++)
    {
      swapped[i] = (swapped[i] << 1) | (shift & 1);
      shift >>= 1;
    }
  }

  // Bit-swap the whole firmware.
  for (int i = 0; i < FLASH_SIZE; i++)
    firmware[i] = swapped[firmware[i]];
}

int write_flash(char *if_name, char *ip_address, int sn)
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
  
  ROS_INFO("******************************************************************");
  ROS_INFO("Firmware file parsed successfully. Will start writing to flash in 5 seconds.");
  ROS_INFO("Hit CTRL+C to abort now.");
  ROS_INFO("******************************************************************");
  sleep(5);
  
  ROS_INFO("Writing to flash. DO NOT ABORT OR TURN OFF CAMERA!!");
  
  unsigned char buff[FLASH_PAGE_SIZE];

  for (int page = 0; page < (firmwarelen + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE; 
      page++)
  {
    if (page % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }

    int addr = page * FLASH_PAGE_SIZE;
    int startretries = 1;
    int retries = startretries;

    if (fcamReliableFlashWrite(camera, page, (uint8_t *) firmware + addr, &retries) != 0)
    {
      ROS_FATAL("Flash write error on page %i.", page);
      ROS_FATAL("If you reset the camera it will probably not come up.");
      ROS_FATAL("Try reflashing NOW, to possibly avoid a hard JTAG reflash.");
      return -2;
    }

    if (retries < startretries)
    {
      fprintf(stderr, "x(%i)", startretries - retries);
    }
  }
  
  fprintf(stderr, "\n");

  ROS_INFO("Verifying flash.");
  
  for (int page = 0; page < (firmwarelen + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE; 
      page++)
  {
    int addr = page * FLASH_PAGE_SIZE;
    if (page % 100 == 0)
    {
      fprintf(stderr, ".");
      fflush(stderr);
    }

    if (fcamReliableFlashRead(camera, page, (uint8_t *) buff, NULL) != 0)
    {
      ROS_FATAL("Flash read error on page %i.", page);
      ROS_FATAL("If you reset the camera it will probably not come up.");
      ROS_FATAL("Try reflashing NOW, to possibly avoid a hard JTAG reflash.");
      return -2;
    }

    if (memcmp(buff, (uint8_t *) firmware + addr, FLASH_PAGE_SIZE))
    {
      ROS_FATAL("Flash compare error on page %i.", page);
      ROS_FATAL("If you reset the camera it will probably not come up.");
      ROS_FATAL("Try reflashing NOW, to possibly avoid a hard JTAG reflash.");
    //  return -2;
    }
  }
  fprintf(stderr, "\n");
  
  ROS_INFO("Success! Restarting camera, should take about 10 seconds to come back up after this.");

  fcamReconfigureFPGA(camera);

  return 0;
}

int main(int argc, char **argv)
{
  if (argc != 5 && argc != 2) {
    fprintf(stderr, "Usage: %s <file.mcs> <Interface> <IP address> <Serial number>\n", argv[0]);
    fprintf(stderr, "       %s <file.mcs> > dump.bin\n", argv[0]);
    return -1;
  }

  if (read_mcs(argv[1]))
    return -1;

  if (firmware[4] == 0x55)
    bitswap();
  else if (firmware[4] != 0xaa)
  {
    ROS_FATAL("Unexpected value at position 4. Don't know whether to bit-swap.");
    return -1;
  }

  if (argc == 2)
  {
    fwrite(firmware, FLASH_SIZE, 1, stdout);
    return 0;
  }

  char* if_name = argv[2];
  char* ip_address = argv[3];
  int sn = atoi(argv[4]);

  write_flash(if_name, ip_address, sn);  
  
}
