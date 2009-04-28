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

#ifndef TRACKER_BASE_H
#define TRACKER_BASE_H

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <opencv_latest/CvBridge.h>
#include <prosilica_cam/PolledImage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>

class TrackerBase
{
public:
  TrackerBase(ros::Node &node, std::string prefix);
  virtual ~TrackerBase();

  void activate();
  void deactivate();

  void spin();
  
protected:
  virtual bool detectObject(tf::Transform &pose) = 0;
  virtual CvRect getBoundingBox() = 0;
  virtual IplImage* getDisplayImage(bool success);

  void processCamInfo();
  void processImage();
  
  CvRect fitToFrame(CvRect roi);
  void setRoi(CvRect roi);
  void setRoiToTargetFrame();

  void saveImage();

  ros::Node &node_;
  boost::thread active_thread_;
  
  prosilica_cam::PolledImage::Request req_;
  prosilica_cam::PolledImage::Response res_;
  std::string image_service_;
  image_msgs::Image& img_;
  image_msgs::CamInfo& cam_info_;
  image_msgs::CvBridge img_bridge_;
  std::string topic_name_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  std::string target_frame_id_;
  std::string object_frame_id_;
  CvMat *K_;

  enum { FullResolution, LastImageLocation, TargetFrame } roi_policy_;
  int frame_w_, frame_h_;
  double resize_factor_found_;
  double resize_factor_failed_;
  int target_roi_size_;
  
  bool display_;
  std::string window_name_;
  int save_count_;
  std::string save_prefix_;
};

#endif
