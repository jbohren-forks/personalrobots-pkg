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

#include <cstdio>

#include "flea2/flea2.h"
#include "ros/node.h"
#include "std_msgs/Image.h"

class Flea2_Node : public ros::Node
{
public:
  std_msgs::Image img;
  Flea2 flea2;

  ros::Time next_time;

  int count;

  Flea2_Node() : ros::Node("flea2")
  {
    advertise<std_msgs::Image>("image");

    flea2.set_shutter(0.8);
    flea2.set_gamma(0.24);
    flea2.set_gain(0);


    next_time = ros::Time::now();
    count = 0;
  }

  bool get_and_send_raw()
  {
    uint8_t *buf;
    uint32_t buf_size;
    uint32_t width;
    uint32_t height;

    flea2.get_frame(&buf, &width, &height);

    buf_size = width*height;
    img.width = width;
    img.height = height;
    img.compression = "raw";
    img.colorspace = "mono8";
    img.set_data_size(buf_size);
    memcpy(&(img.data[0]), buf, buf_size);

    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " imgs/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    publish("image", img);
    return true;
  }
  
  bool get_and_send_jpeg() 
  {
    const uint8_t *buf;
    uint32_t buf_size;
    flea2.get_jpeg(&buf, &buf_size);

    img.width = flea2.cam.frame_width;
    img.height = flea2.cam.frame_height;
    img.compression = "jpeg";
    img.colorspace = "mono8";
    img.set_data_size(buf_size);
    memcpy(&(img.data[0]), buf, buf_size);


    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " imgs/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    publish("image", img);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  
  Flea2_Node fn;

  while (fn.ok()) {
    //fn.get_and_send_jpeg();
    fn.get_and_send_raw();
  }

  ros::fini();
  return 0;
}

