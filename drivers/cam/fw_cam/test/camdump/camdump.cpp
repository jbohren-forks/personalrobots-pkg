///////////////////////////////////////////////////////////////////////////////
// The flea2 provides a bare-bones driver for the flea2 camera
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream.h>
#include <fstream.h>
#include "fw_cam.h"
#include "ros/time.h"


int main(int argc, char **argv)
{

  FwHost fw1(0);
  FwHost fw2(1);
  FwCam cam1(&fw1, 0, 0);
  FwCam cam2(&fw1, 1, 1);
  FwCam cam3(&fw2, 0, 0);
  FwCam cam4(&fw2, 1, 1);

  ofstream file("camdump.raw", fstream::out | fstream::binary);

  uint8_t *buf1;
  uint8_t *buf2;
  uint8_t *buf3;
  uint8_t *buf4;

  uint32_t width;
  uint32_t height;

  ros::Time next_time = ros::Time::now() + ros::Duration(1,0);
  int count = 0;

  for (int i = 0; i < 100000; i++)
  {
    cam1.get_frame(&buf1, &width, &height);
    cam2.get_frame(&buf2, &width, &height);
    cam3.get_frame(&buf3, &width, &height);
    cam4.get_frame(&buf4, &width, &height);

    file.write((char*)(buf1), width*height);
    file.write((char*)(buf2), width*height);
    file.write((char*)(buf3), width*height);
    file.write((char*)(buf4), width*height);

    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << "  fps at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }
  }

  file.close();
  return 0;
}

