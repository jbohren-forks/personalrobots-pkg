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

#include "ros/node.h"

#include "image.h"
#include "cam_bridge.h"

#include "image_msgs/Image.h"
#include "image_msgs/FillImage.h"
#include "image_msgs/CamInfo.h"

#include <boost/thread.hpp>

#include "diagnostic_updater/diagnostic_updater.h"

using namespace std;

class ImageProc
{
  bool do_colorize_;
  bool do_rectify_;

  image_msgs::Image raw_img_;
  image_msgs::CamInfo cam_info_;
  bool have_cam_info_;
  boost::mutex cam_info_mutex_;

  image_msgs::Image img_;

  //DiagnosticUpdater<ImageProc> diagnostic_;
  int count_;
  double desired_freq_;

  std::string cam_name_;

  ros::Node &node_;

public:

  cam::ImageData img_data_;

  ImageProc(ros::Node &node) : have_cam_info_(false), count_(0), node_(node)
  {
    cam_name_ = node_.mapName("camera") + "/";
    node_.param(cam_name_ + "do_colorize", do_colorize_, false);
    node_.param(cam_name_ + "do_rectify", do_rectify_, false);

    advertiseImages();
    node_.subscribe(cam_name_ + "cam_info", cam_info_, &ImageProc::camInfoCb, this, 1);
    node_.subscribe(cam_name_ + "image_raw", raw_img_, &ImageProc::rawCb, this, 1);
  }

  // TODO: should the callbacks actually be synchronized? is there a use case?
  void camInfoCb()
  {
    boost::lock_guard<boost::mutex> guard(cam_info_mutex_);
    have_cam_info_ = true;
  }
  
  void rawCb()
  {
    boost::lock_guard<boost::mutex> guard(cam_info_mutex_);

    cam_bridge::RawStereoToCamData(raw_img_, cam_info_, image_msgs::RawStereo::IMAGE_RAW, &img_data_);
    img_data_.imRawType = COLOR_CODING_BAYER8_BGGR;

    if (do_colorize_) {
      //img_data_.colorConvertType = COLOR_CONVERSION_EDGE;
      img_data_.doBayerColorRGB();
      //img_data_.doBayerMono();
    }

    if (do_rectify_ && have_cam_info_)
      img_data_.doRectify();

    publishImages();

    count_++;
  }

  void publishImages()
  {
    img_.header.stamp = raw_img_.header.stamp;
    img_.header.frame_id = raw_img_.header.frame_id;
    
    if (img_data_.imType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image",
                img_data_.imHeight, img_data_.imWidth, 1,
                "mono", "uint8",
                img_data_.im );
      node_.publish(cam_name_ + "image", img_);
    }

    if (img_data_.imColorType == COLOR_CODING_RGB8)
    {
      fillImage(img_,  "image_color",
                img_data_.imHeight, img_data_.imWidth, 3,
                "rgb", "uint8",
                img_data_.imColor );
      node_.publish(cam_name_ + "image_color", img_);
    }

    if (img_data_.imRectType != COLOR_CODING_NONE)
    {
      fillImage(img_,  "image_rect",
                img_data_.imHeight, img_data_.imWidth, 1,
                "mono", "uint8",
                img_data_.imRect );
      node_.publish(cam_name_ + "image_rect", img_);
    }

    if (img_data_.imRectColorType == COLOR_CODING_RGB8)
    {
      fillImage(img_,  "image_rect_color",
                img_data_.imHeight, img_data_.imWidth, 3,
                "rgb", "uint8",
                img_data_.imRectColor );
      node_.publish(cam_name_ + "image_rect_color", img_);
    }

    if (img_data_.imRectColorType == COLOR_CODING_RGBA8)
    {
      fillImage(img_,  "image_rect_color",
                img_data_.imHeight, img_data_.imWidth, 4,
                "rgba", "uint8",
                img_data_.imRectColor );
      node_.publish(cam_name_ + "image_rect_color", img_);
    }
  }


  void advertiseImages()
  {
    node_.advertise<image_msgs::Image>(cam_name_ + "image", 1);
    node_.advertise<image_msgs::Image>(cam_name_ + "image_color", 1);
    node_.advertise<image_msgs::Image>(cam_name_ + "image_rect", 1);
    node_.advertise<image_msgs::Image>(cam_name_ + "image_rect_color", 1);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("stereoproc");
  ImageProc ip(n);

  n.spin();

  return 0;
}
