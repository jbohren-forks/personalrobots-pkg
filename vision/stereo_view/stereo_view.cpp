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

#include "image_msgs/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/Image.h"


#include "color_calib.h"

#include "topic_synchronizer.h"

#include <boost/thread.hpp>

using namespace std;

class StereoView : public ros::Node
{
public:

  image_msgs::Image limage;
  image_msgs::Image rimage;
  image_msgs::Image dimage;
  image_msgs::StereoInfo stinfo;
  image_msgs::DisparityInfo dinfo;

  image_msgs::CvBridge lbridge;
  image_msgs::CvBridge rbridge;
  image_msgs::CvBridge dbridge;

  color_calib::Calibration lcal;
  color_calib::Calibration rcal;

  IplImage* lcalimage;
  IplImage* rcalimage;

  TopicSynchronizer<StereoView> sync;

  bool subscribe_color_;
  bool calib_color_;
  bool recompand_;

  boost::mutex cv_mutex;

  StereoView() : ros::Node("stereo_view"), 
                 lcal(this), rcal(this), lcalimage(NULL), rcalimage(NULL),
                 sync(this, &StereoView::image_cb_all, ros::Duration().fromSec(0.05), &StereoView::image_cb_timeout),
                 calib_color_(false), recompand_(false)
  { 
    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);

    std::list<std::string> left_list;
    left_list.push_back(mapName("stereo") + std::string("/left/image_rect_color"));
    left_list.push_back(mapName("stereo") + std::string("/left/image_rect"));


    std::list<std::string> right_list;
    right_list.push_back(mapName("stereo") + std::string("/right/image_rect_color"));
    right_list.push_back(mapName("stereo") + std::string("/right/image_rect"));

    sync.subscribe(left_list,  limage, 1);
    sync.subscribe(right_list, rimage, 1);

    sync.subscribe(mapName("stereo") + "/disparity", dimage, 1);
    sync.subscribe(mapName("stereo") + "/stereo_info", stinfo, 1);
    sync.subscribe(mapName("stereo") + "/disparity_info", dinfo, 1);

    sync.ready();
  }

  ~StereoView()
  {
    if (lcalimage)
      cvReleaseImage(&lcalimage);
    if (rcalimage)
      cvReleaseImage(&rcalimage);
  }

  void image_cb_all(ros::Time t)
  {
    cv_mutex.lock();

    if (lbridge.fromImage(limage, "bgr"))
    {
      if (calib_color_)
      {
        lbridge.reallocIfNeeded(&lcalimage, IPL_DEPTH_32F);

        lcal.correctColor(lbridge.toIpl(), lcalimage, true, recompand_, COLOR_CAL_BGR);

        cvShowImage("left", lcalimage);
      } else
        cvShowImage("left", lbridge.toIpl());
    }

    if (rbridge.fromImage(rimage, "bgr"))
    {
      if (calib_color_)
      {
        rbridge.reallocIfNeeded(&rcalimage, IPL_DEPTH_32F);

        rcal.correctColor(rbridge.toIpl(), rcalimage, true, recompand_, COLOR_CAL_BGR);

      cvShowImage("right", rcalimage);
      } else
        cvShowImage("right", rbridge.toIpl());
    }

    if (dbridge.fromImage(dimage))
    {
      // Disparity has to be scaled to be be nicely displayable
      IplImage* disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
      cvCvtScale(dbridge.toIpl(), disp, 4.0/dinfo.dpp);
      cvShowImage("disparity", disp);
      cvReleaseImage(&disp);
    }

    cv_mutex.unlock();

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
  
  bool spin()
  {
    while (ok())
    {
      cv_mutex.lock();
      int key = cvWaitKey(3);
      
      switch (key) {
      case 10:
        calib_color_ = !calib_color_;
        break;
      case 32:
        recompand_ = !recompand_;
      }

      // Fetch color calibration parameters as necessary
      if (calib_color_)
      {
        lcal.getFromParam(mapName("stereo") + "/left/image_rect_color");
        rcal.getFromParam(mapName("stereo") + "/right/image_rect_color");
      }

      cv_mutex.unlock();
      usleep(10000);
    }

    return true;
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

