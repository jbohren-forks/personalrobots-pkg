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

#include "ros/node.h"
#include "std_msgs/Image.h"
#include "axis_cam/PTZActuatorCmd.h"
#include "axis_cam/PTZActuatorState.h"
#include "image_utils/cv_bridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


class AxisDemo : public ros::Node
{
public:
  std_msgs::Image image_;
  CvBridge<std_msgs::Image> cv_bridge_;

  axis_cam::PTZActuatorCmd   ptz_cmd_;
  axis_cam::PTZActuatorState ptz_state_;

  IplImage *cv_image_;

  AxisDemo() : ros::Node("axis_demo"), cv_bridge_(&image_, CvBridge<std_msgs::Image>::CORRECT_BGR), cv_image_(NULL)
  {
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);

    subscribe("image", image_, &AxisDemo::image_cb, 1);
    subscribe("ptz_state", ptz_state_, &AxisDemo::ptz_cb, 1);

    advertise<axis_cam::PTZActuatorCmd>("ptz_cmd", 1);

    ptz_cmd_.pan.cmd   = 50;
    ptz_cmd_.pan.rel   = 0;
    ptz_cmd_.pan.mode  = 0;
    ptz_cmd_.pan.valid = 1;
  }

  ~AxisDemo()
  {
    if (cv_image_)
      cvReleaseImage(&cv_image_);
  }

  void image_cb()
  {
    printf("Received %dx%d image\n", image_.width, image_.height);

    if (cv_image_)
      cvReleaseImage(&cv_image_);

    if (cv_bridge_.to_cv(&cv_image_))
      cvShowImage("cv_view", cv_image_);

    cvWaitKey(3);
  }

  void ptz_cb()
  {
    printf("Received axis state:\n");
    printf(" Pan:  %f\n", ptz_state_.pan.pos);
    printf(" Tilt: %f\n", ptz_state_.tilt.pos);
    printf(" Zoom: %f\n\n", ptz_state_.zoom.pos);
  }


  bool spin()
  {
    while (ok())
    {
      ptz_cmd_.pan.cmd = -ptz_cmd_.pan.cmd;
      publish("ptz_cmd", ptz_cmd_);
      usleep(5000000);
    }
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  AxisDemo a;

  a.spin();

  ros::fini();

  return 0;
}
