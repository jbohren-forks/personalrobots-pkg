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

#ifndef OUTLET_TRACKER_H
#define OUTLET_TRACKER_H

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <opencv_latest/CvBridge.h>
#include <robot_msgs/PoseStamped.h>
#include <prosilica_cam/PolledImage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>

#include <boost/thread.hpp>

#include "outlet_detection/outlet_detector.h"


class OutletTracker
{
public:
  OutletTracker(ros::Node &node);
  ~OutletTracker();

  void activate();
  void deactivate();

  void spin();
  
private:
  void processCamInfo();
  void processImage();

  bool detectOutlet(robot_msgs::PoseStamped &pose);

  // TODO: separate out these functions, shared with plug_tracker
  CvRect fitToFrame(CvRect roi);
  void setRoi(CvRect roi);

  ros::Node &node_;
  boost::thread active_thread_;

  prosilica_cam::PolledImage::Request req_;
  prosilica_cam::PolledImage::Response res_;
  std::string image_service_;
  image_msgs::Image& img_;
  image_msgs::CamInfo& cam_info_;
  image_msgs::CvBridge img_bridge_;
  std::vector<outlet_t> outlets_;
  tf::TransformBroadcaster tf_broadcaster_;
  CvMat* K_;
  int display_;

  // TODO: policy to listen from coarse outlet detection?
  enum { WholeFrame, LastImageLocation } roi_policy_;
  int frame_w_, frame_h_;
  static const float RESIZE_FACTOR = 1.2f;

  static const char wndname[];
};

#endif
