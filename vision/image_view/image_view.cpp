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

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <opencv_latest/CvBridge.h>

#include <boost/thread.hpp>

class ImageView
{
private:
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  std::string window_name_;
  std::string filename_format_;
  int count_;
  bool save_;
  boost::mutex save_mutex_;

public:
  ImageView() : count_(0), save_(false)
  {
    ros::Node* node = ros::Node::instance();

    node->param("~window_name", window_name_, node->mapName("image"));
    bool autosize;
    node->param("~autosize", autosize, true);
    node->param("~filename_format", filename_format_, std::string("frame%04i.png"));
    
    cvNamedWindow(window_name_.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvSetMouseCallback(window_name_.c_str(), &ImageView::mouse_cb, this);
    cvStartWindowThread();

    node->subscribe("image", img_msg_, &ImageView::image_cb, this, 1);
  }

  ~ImageView()
  {
    cvDestroyWindow(window_name_.c_str());
  }

  void image_cb()
  {
    // May want to view raw bayer data
    if (img_msg_.encoding.find("bayer") != std::string::npos)
      img_msg_.encoding = "mono";
    
    if (img_bridge_.fromImage(img_msg_, "bgr"))
      cvShowImage(window_name_.c_str(), img_bridge_.toIpl());

    boost::lock_guard<boost::mutex> guard(save_mutex_);
    if (save_) {
      char filename[256];
      if (snprintf(filename, sizeof(filename), filename_format_.c_str(),
                   count_++) < (int)sizeof(filename)) {
        cvSaveImage(filename, img_bridge_.toIpl());
        ROS_INFO("Saved image %s", filename);
      } else {
        ROS_WARN("Couldn't save image, file name too long!");
      }
      save_ = false;
    }
  }

  static void mouse_cb(int event, int x, int y, int flags, void* param)
  {
    if (event != CV_EVENT_LBUTTONDOWN)
      return;
    
    ImageView *iv = (ImageView*)param;
    iv->save_mutex_.lock();
    iv->save_ = true;
    iv->save_mutex_.unlock();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("image_view");
  ImageView view;
  n.spin();
  
  return 0;
}
