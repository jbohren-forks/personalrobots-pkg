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
#include "std_msgs/ImageArray.h"
#include "image_utils/cv_bridge.h"

#include <sys/stat.h>

#include "colorcalib.h"

using namespace std;

class ColorCalib : public ros::node
{
public:
  std_msgs::ImageArray image_msg;

  ros::thread::mutex cv_mutex;

  CvMat* color_cal;

  ColorCalib() : node("color_calib", ros::node::ANONYMOUS_NAME)
  { 
    subscribe("images", image_msg, &ColorCalib::image_cb, 1);
    color_cal = cvCreateMat( 3, 3, CV_32FC1);
  }

  ~ColorCalib()
  {
    cvReleaseMat(&color_cal);
  }

  void image_cb()
  {
    unsubscribe("images");

    cv_mutex.lock();

    for (uint32_t i = 0; i < image_msg.get_images_size(); i++)
    {
      string l = image_msg.images[i].label;

      if (image_msg.images[i].colorspace == std::string("rgb24"))
      {
        CvBridge<std_msgs::Image>* cv_bridge = new CvBridge<std_msgs::Image>(&image_msg.images[i], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
        IplImage* img;

        if (cv_bridge->to_cv(&img))
        {
          IplImage* img2 = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 3);

          decompand(img, img2);

          find_calib(img2, color_cal, COLOR_CAL_BGR);
          
          printf("Color calibration:\n");
          for (int i = 0; i < 3; i ++)
          {
            for (int j = 0; j < 3; j++)
            {
              printf("%f ", cvmGet(color_cal, i, j));
            }
            printf("\n");
          }

          IplImage* corrected_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 3);
          cvTransform(img2, corrected_img, color_cal);

          cvNamedWindow("color_rect", CV_WINDOW_AUTOSIZE);
          cvShowImage("color_rect", corrected_img);
        }

        delete cv_bridge;
      }
    }
    cv_mutex.unlock();
  }

  void check_keys() 
  {
    cv_mutex.lock();
    if (cvWaitKey(3) == 10)
    { }

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
