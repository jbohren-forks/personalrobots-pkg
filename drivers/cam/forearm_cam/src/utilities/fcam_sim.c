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

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ipcam_packet.h>

int main(int argc, char **argv)
{
  int udp_socket[5];
  struct sockaddr_in to, from;
  PacketVideoLine pvl;
	PacketEOF peof;
  int line;
  int frame = 0xFF00;
  int i;
  int cursocket = 0;

  if (argc != 2)
  {
    printf("Bad args : udpspew port\n");
    return 1;
  }

  from.sin_family = AF_INET;
	from.sin_addr.s_addr = inet_addr("192.168.201.2");
  for (i = 0; i < 5; i++)
  {
    udp_socket[i] = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    from.sin_port = htons(10000 + i);
  
    if (bind(udp_socket[i], (struct sockaddr *) &from, sizeof(from))) 
    {
      perror("bind"); 
      exit(1); 
    }
  }
  
	to.sin_family = AF_INET;
	to.sin_port = htons(atoi(argv[1]));
	to.sin_addr.s_addr = inet_addr("192.168.201.1");
  
#define XSIZE 640
#define YSIZE 480

  peof.header.horiz_resolution = pvl.header.horiz_resolution = htons(XSIZE);
  peof.header.vert_resolution = pvl.header.vert_resolution = htons(YSIZE);
  
  while (1)
	{
    frame++;
    frame = frame & 0xFFFF;
    peof.header.frame_number = pvl.header.frame_number = htonl(frame);
    for (line = 0; line < YSIZE; line++)
    {
      pvl.header.line_number = htons(line | (frame << 10));
      while (sendto(udp_socket[cursocket], &pvl, sizeof(pvl.header) + XSIZE, 0, (struct sockaddr *) &to, sizeof(to)) == -1)
      {}
      usleep(50);
      cursocket = (cursocket + 1) & 3;
    }
    
    peof.header.line_number = htons(IMAGER_LINENO_EOF);
    while (sendto(udp_socket[4], &peof, sizeof(peof), 0, (struct sockaddr *) &to, sizeof(to)) == -1)
    {}
    usleep(4000);

    if ((frame & 0x7F) == 0)
		{
			printf("%x\n", frame);
			fflush(stdout);
		}
	}

  return 0;
}
