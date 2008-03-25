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
#include "axis213/axis213.h"

#include "ros/ros_slave.h"
#include "common_flows/FlowEmpty.h"

#include "image_flows/FlowImage.h"
#include "image_flows/image_flow_codec.h"



class Axis213_cam : public ROS_Slave
{
public:
  FlowImage *image_all;
  FlowImage *image_polled;

  FlowEmpty *shutter;

  string axis_host;
  Axis213 *cam;

  Axis213_cam() : ROS_Slave(), cam(NULL)
  {
    register_source(image_all = new FlowImage("image_all"));

    register_source(image_polled = new FlowImage("image_polled"));

    register_sink(shutter = new FlowEmpty("shutter"), ROS_CALLBACK(Axis213_cam, shutter_callback));

    if (!get_string_param(".host", axis_host))
    {
      printf("axis_host parameter not specified; defaulting to 10.0.0.150\n");
      axis_host = "10.0.0.150";
    }
    printf("axis host set to [%s]\n", axis_host.c_str());
    cam = new Axis213(axis_host);

    register_with_master();
  }

  virtual ~Axis213_cam()
  { 
    if (cam) 
      delete cam; 
  }

  bool take_and_send_image()
  {
    uint8_t *jpeg;
    uint32_t jpeg_size;
    if (!cam->get_jpeg(&jpeg, &jpeg_size))
      return false;
    //image_all->set_jpeg_buffer(jpeg, jpeg_size, 704, 480);
    image_all->width = 704;
    image_all->height = 480;
    image_all->compression = "jpeg";
    image_all->colorspace = "rgb24";
    image_all->frame_id = 0;
    image_all->set_data_size(jpeg_size);
    memcpy(image_all->data, jpeg, jpeg_size);
    image_all->publish();
    return true;
  }

  void shutter_callback()
  {
    image_all->lock_atom();
    //    image_polled->set_jpeg_buffer(image_all->jpeg_buffer, image_all->compressed_size, 704, 480);
    
    image_all->unlock_atom();
    image_polled->width = image_all->width;
    image_polled->height = image_all->height;
    image_polled->compression = image_all->compression;
    image_polled->colorspace = image_polled->colorspace;
    image_polled->frame_id = image_all->frame_id;
    image_polled->set_data_size(image_all->get_data_size());
    memcpy(image_polled->data, image_all->data, image_all->get_data_size());
    image_polled->publish();
  }

};

int main(int argc, char **argv)
{
  Axis213_cam a;
  while (a.happy())
  {
    if (!a.take_and_send_image())
      {
	printf("couldn't take image.\n");
	break;
      }

  }
  return 0;
}

