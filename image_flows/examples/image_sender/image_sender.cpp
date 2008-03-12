///////////////////////////////////////////////////////////////////////////////
// The image_flows package provides image transport, codec wrapping, and 
// various handy image utilities.
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

#include <cstdio>
#include "image_flows/FlowImage.h"
#include "image_flows/image_flow_codec.h"
#include "ros/ros_slave.h"

class ImageSender : public ROS_Slave
{
public:
  double freq;
  string image_file;
  FlowImage *image;
  ImageFlowCodec<FlowImage> *codec;

  ImageSender() : ROS_Slave(), freq(1), image_file("test.jpg")
  {
    register_source(image = new FlowImage("image"));
    codec = new ImageFlowCodec<FlowImage>(image);
    register_with_master();
    get_double_param(".freq", &freq);
    get_string_param(".image_file", image_file);
    codec->read_file(image_file);
    printf("WD is [%s]\n", getcwd(NULL, 0));
  }
  void send_image()
  {
    printf("sending image %d\n", image->publish_count);
    image->publish();
  }
};

int main(int argc, char **argv)
{
  ImageSender is;
  int sleep_usecs = (int)(1000000.0 / is.freq);
  while (is.happy()) // ha ha CODING PUN
  {
    usleep(sleep_usecs);
    is.send_image();
  }
  return 0;
}
