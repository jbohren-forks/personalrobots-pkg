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

#include <vector>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"

#include "topic_synchronizer.h"

using namespace std;

class StereoView : public ros::node
{
public:

  image_msgs::Image limage;
  image_msgs::Image rimage;
  image_msgs::Image dimage;
  image_msgs::StereoInfo stinfo;

  image_msgs::CvBridge lbridge;
  image_msgs::CvBridge rbridge;
  image_msgs::CvBridge dbridge;

  TopicSynchronizer<StereoView> sync;

  bool subscribe_color_;

  StereoView() : ros::node("stereo_view"), sync(this, &StereoView::image_cb_all, ros::Duration(0.05), &StereoView::image_cb_timeout), subscribe_color_(false)
  { 
    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);

    param("~subscribe_color", subscribe_color_, false);
    
    if (subscribe_color_)
    {
      sync.subscribe("dcam/left/image_rect_color", limage, 1);
      sync.subscribe("dcam/right/image_rect_color", rimage, 1);
    } else {
      sync.subscribe("dcam/left/image_rect", limage, 1);
      sync.subscribe("dcam/right/image_rect", rimage, 1);
    }
    sync.subscribe("dcam/disparity", dimage, 1);
    sync.subscribe("dcam/stereo_info", stinfo, 1);
  }

  void image_cb_all(ros::Time t)
  {
    if (lbridge.fromImage(limage, "bgr"))
      cvShowImage("left", lbridge.toIpl());

    if (rbridge.fromImage(rimage, "bgr"))
      cvShowImage("right", rbridge.toIpl());

    if (dbridge.fromImage(dimage))
    {
      // Disparity has to be scaled to be be nicely displayable
      IplImage* disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
      cvCvtScale(dbridge.toIpl(), disp, 4.0/stinfo.dpp);
      cvShowImage("disparity", disp);
      cvReleaseImage(&disp);
    }

    cvWaitKey(5);
  }

  void image_cb_timeout(ros::Time t)
  {
    if (limage.header.stamp != t)
      printf("Timed out waiting for left image\n");

    if (rimage.header.stamp != t)
      printf("Timed out waiting for right image\n");

    if (dimage.header.stamp != t)
      printf("Timed out waiting for disparity image\n");

    //Proceed to show images anyways
    image_cb_all(t);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StereoView view;
  view.spin();
  ros::fini();
  return 0;
}

