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

#include <stdio.h>
#include <iostream>

#include <pthread.h>

#include "ros/node.h"
#include "std_msgs/Image.h"
#include "std_srvs/PolledImage.h"
#include "image_utils/image_codec.h"

#include "axis_cam/axis_cam.h"

#include "self_test/self_test.h"

class Axis_cam_node : public ros::node
{
public:
  std_msgs::Image image;
  ImageCodec<std_msgs::Image> codec;

  string axis_host;
  AxisCam *cam;
  int frame_id;

  SelfTest<Axis_cam_node> self_test_;

  ros::Time next_time;
  int count_;

  Axis_cam_node() : ros::node("axis_cam"), codec(&image), cam(NULL), frame_id(0), self_test_(this)
  {
    advertise<std_msgs::Image>("image");
    advertise_service("polled_image", &Axis_cam_node::polled_image_cb);

    param("~host", axis_host, string("192.168.0.90"));
    printf("axis_cam host set to [%s]\n", axis_host.c_str());

    self_test_.lock();

    self_test_.addTest(&Axis_cam_node::checkMac);

    cam = new AxisCam(axis_host);

    next_time = ros::Time::now();
    count_ = 0;
    
    self_test_.unlock();
  }

  virtual ~Axis_cam_node()
  { 
    if (cam) 
      delete cam; 
  }

  bool polled_image_cb(std_srvs::PolledImage::request  &req,
                       std_srvs::PolledImage::response &res )
  {
    image.lock();
    res.image = image;
    image.unlock();
    return true;
  }

  bool take_and_send_image()
  {
    self_test_.lock();
    uint8_t *jpeg;
    uint32_t jpeg_size;

    if (!cam->get_jpeg(&jpeg, &jpeg_size))
    {
      log(ros::ERROR, "woah! AxisCam::get_jpeg returned an error");
      self_test_.unlock();
      sched_yield();
      return false;
    }

    image.set_data_size(jpeg_size);
    memcpy(image.data, jpeg, jpeg_size);

    image.compression = "jpeg";

    codec.inflate_header();

    publish("image", image);

    self_test_.unlock();

    sched_yield();

    return true;
  }


  bool spin()
  {

    while (ok())
    {
      if (take_and_send_image())
      {
        count_++;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time) {
          std::cout << count_ << " frames/sec at " << now_time << std::endl;
          count_ = 0;
          next_time = next_time + ros::Duration(1,0);
        }
      } else {
        log(ros::ERROR,"couldn't take image.");
        usleep(1000000);
        param("~host", axis_host, string("192.168.0.90"));
        cam->set_host(axis_host);
      }
    }
    return true;
  }


  void checkImage(robot_msgs::DiagnosticStatus& status)
  { 
   
  }

  void checkMac(robot_msgs::DiagnosticStatus& status)
  {
    char cmd[100];
    snprintf(cmd, 100, "arp -n %s", axis_host.c_str());
    
    FILE* f = popen(cmd, "r");
    char buf1[100];
    char buf2[100];
    char buf3[100];
    char buf4[100];

    if (fscanf(f, "Address HWtype HWaddress Flags Mask Iface\n%s %s %s %s", buf1, buf2, buf3, buf4) < 4)
    {
      status.level = 2;
      status.message = "No mac address found in ARP table.";
    }
    else
    {
      status.level = 0;
      status.message = buf3;
      self_test_.setID(buf3);
    }
    fclose(f);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  Axis_cam_node a;

  a.spin();

  ros::fini();

  return 0;
}
