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

#include <cstdio>
#include <vector>
#include <map>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"

#include <sys/stat.h>

#include "color_calib.h"

using namespace std;
using namespace color_calib;

class ColorCalib : public ros::node
{
public:
  image_msgs::Image image;
  image_msgs::CvBridge bridge;

  ros::thread::mutex cv_mutex;

  Calibration color_cal;

  bool first;

  ColorCalib() : node("color_calib", ros::node::ANONYMOUS_NAME), color_cal(this), first(true)
  { 
    subscribe("image", image, &ColorCalib::image_cb, 1);
  }

  void image_cb()
  {
    if (!first)
      return;
    else
      first = false;

    //    unsubscribe("image");

    cv_mutex.lock();

    if (bridge.fromImage(image, "bgr"))
    {
      IplImage* cv_img  = bridge.toIpl();
      IplImage* cv_img_decompand = cvCreateImage(cvGetSize(cv_img), IPL_DEPTH_32F, 3);
      decompand(cv_img, cv_img_decompand);

      find_calib(cv_img_decompand, color_cal, COLOR_CAL_BGR | COLOR_CAL_COMPAND_DISPLAY);

      color_cal.setParam("image");

      IplImage* cv_img_correct = cvCreateImage(cvGetSize(cv_img), IPL_DEPTH_32F, 3);
      cvTransform(cv_img_decompand, cv_img_correct, color_cal.getCal(COLOR_CAL_BGR));
      
      cvNamedWindow("color_rect", CV_WINDOW_AUTOSIZE);
      cvShowImage("color_rect", cv_img_correct);

      compand(cv_img_correct, cv_img_correct);
      cvNamedWindow("color_rect_compand", CV_WINDOW_AUTOSIZE);
      cvShowImage("color_rect_compand", cv_img_correct);

      cvReleaseImage(&cv_img_decompand);
      cvReleaseImage(&cv_img_correct);
    }

    cv_mutex.unlock();
  }

  void check_keys() 
  {
    cv_mutex.lock();

    if (cvWaitKey(3) == 10)
      self_destruct();

    cv_mutex.unlock();
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ColorCalib view;
  while (view.ok()) 
  {
    usleep(10000);
    view.check_keys();
  }
  ros::fini();
  return 0;
}
