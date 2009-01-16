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


#include <stdio.h>
#include <stdlib.h>
#include <dc1394_cam/dc1394_cam.h>

#include <fstream>
#include <vector>

using namespace std;

int main(int argc, char *argv[])
{

  if (argc < 2)
  {
    printf("Please specify a filename.\n");
    return 1;
  }

  ofstream file1(argv[1], fstream::out | fstream::binary);

  if (!file1.is_open()) {
    printf("Could not open file.\n");
    return 1;
  }

  dc1394_cam::init();

  int num = dc1394_cam::numCams();

  if (num > 0)
  {
    printf("Found %d cameras\n", num);
  } else {
    printf("No cameras found.");
    return 1;
  }

  vector<dc1394_cam::Cam*> cameras;

  for (int i = 0; i < num; i++) 
  {
    uint64_t guid = dc1394_cam::getGuid(i);
    dc1394_cam::Cam* c = new dc1394_cam::Cam(guid,
                                             DC1394_ISO_SPEED_400,
                                             DC1394_VIDEO_MODE_640x480_MONO8,
                                             DC1394_FRAMERATE_30,
                                             45);
    printf("Opening camera with GUID %llx\n", guid);
    cameras.push_back(c);
  }

  struct timespec start_time;
  clock_gettime(CLOCK_REALTIME, &start_time);

  struct timespec next_time;
  clock_gettime(CLOCK_REALTIME, &next_time);

  next_time.tv_sec += 1;

  int count = 0;
    

  printf("trying to capture frames...\n");
  for (int i = 0; i < 100000; i++) {
    for (vector<dc1394_cam::Cam*>::iterator c = cameras.begin(); c != cameras.end(); c++)
    {
      dc1394_cam::FrameSet fs = (*c)->getFrames();
      
      if (fs.size() > 0)
      {
        if (fs[0].getFrame()->frames_behind >= 44)
          fprintf(stderr, "\nDropped frames (Behind by %d) time: %d secs\n", fs[0].getFrame()->frames_behind, next_time.tv_sec - start_time.tv_sec);
        
        file1.write((char*)(fs[0].getFrame()->image), fs[0].getFrame()->image_bytes);
        
        fs[0].releaseFrame();
        
        count++;
        
        if (count % 1000 == 0)
          fprintf(stderr,".");
        if (count % 100000 == 0)
          fprintf(stderr,"\n");
        
      }
      
      struct timespec now_time;
      clock_gettime(CLOCK_REALTIME, &now_time);
      
      if (now_time.tv_sec >= next_time.tv_sec) {
        next_time.tv_sec++;
      }
    }
  }

  file1.close();

  for (vector<dc1394_cam::Cam*>::iterator c = cameras.begin(); c != cameras.end(); c++)
  {
    delete *c;
  }
    

  dc1394_cam::fini();

  return 0;
}
