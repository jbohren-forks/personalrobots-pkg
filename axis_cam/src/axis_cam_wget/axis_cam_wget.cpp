///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
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

#include "ros/ros_slave.h"
#include "SDL/SDL.h"
#include "image_flows/FlowImage.h"
#include "axis_cam/axis_cam.h"

class AxisCamWget : public ROS_Slave
{
public:
  FlowImage *image;
  string axis_host;
  AxisCam *cam;
  int frame_id;

  AxisCamWget() : ROS_Slave(), cam(NULL), frame_id(0)
  {
    register_source(image = new FlowImage("image"));
    if (!get_string_param(".host", axis_host))
    {
      printf("host parameter not specified; defaulting to 192.168.0.90\n");
      axis_host = "192.168.0.90";
    }
    printf("axis_cam host set to [%s]\n", axis_host.c_str());
    get_int_param(".frame_id", &frame_id);
    cam = new AxisCam(axis_host);
  }
  virtual ~AxisCamWget()
  { 
    if (cam) 
      delete cam; 
  }
  bool take_and_send_image()
  {
    uint8_t *jpeg;
    uint32_t jpeg_size;
    if (!cam->wget_jpeg(&jpeg, &jpeg_size))
      return false;
    image->set_data_size(jpeg_size);
    memcpy(image->data, jpeg, jpeg_size);
    image->width = 704;
    image->height = 480;
    image->compression = "jpeg";
    image->colorspace = "rgb24";
    image->frame_id = frame_id;
    image->publish();
    return true;
  }
};

int main(int argc, char **argv)
{
  AxisCamWget a;
  while (a.happy())
    if (!a.take_and_send_image())
    {
      printf("couldn't take image.\n");
      break;
    }
  return 0;
}

