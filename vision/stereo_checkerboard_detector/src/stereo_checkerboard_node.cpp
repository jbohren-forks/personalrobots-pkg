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

//! \author Vijay Pradeep


#include "ros/node.h"
#include "stereo_checkerboard_detector/stereo_checkerboard_helper.h"

#include "robot_msgs/PointCloud.h"

#include "topic_synchronizer/topic_synchronizer.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;

class StereoCheckerboardNode
{

public:
  StereoCheckerboardNode(ros::Node* node) : node_(node),
  sync_(node_, this, &StereoCheckerboardNode::msgCallback,
        ros::Duration().fromSec(0.05),
        &StereoCheckerboardNode::msgTimeout )
  {
    cb_helper_.setSize(6,8) ;

    sync_.subscribe("stereo/left/image_rect", left_image_msg_,  1) ;
    sync_.subscribe("stereo/left/cam_info",  left_info_msg_,   1) ;
    sync_.subscribe("stereo/right/image_rect",right_image_msg_, 1) ;
    sync_.subscribe("stereo/right/cam_info", right_info_msg_,  1) ;

    node_->advertise<robot_msgs::PointCloud>("cb_corners", 1) ;
    node_->advertise<image_msgs::Image>("left_cb_debug",1) ;
    node_->advertise<image_msgs::Image>("right_cb_debug",1) ;

    sync_.ready() ;
  }

  ~StereoCheckerboardNode()
  {

  }

  void msgCallback(ros::Time t)
  {
    bool found = cb_helper_.findCheckerboard(left_image_msg_, right_image_msg_, left_info_msg_, right_info_msg_) ;
    if (found)
    {
      vector<robot_msgs::Point> xyz ;
      cb_helper_.getCorners(xyz) ;

      unsigned int N = xyz.size() ;

      robot_msgs::PointCloud cloud ;
      cloud.pts.resize(N) ;
      for(unsigned int i=0; i<N; i++)
      {
        cloud.pts[i].x = xyz[i].x ;
        cloud.pts[i].y = xyz[i].y ;
        cloud.pts[i].z = xyz[i].z ;
      }
      cloud.header.frame_id = left_image_msg_.header.frame_id ;
      cloud.header.stamp = left_image_msg_.header.stamp ;
      node_->publish("cb_corners", cloud) ;
      printf("Found CB!\n") ;
    }
    else
      printf("Not found\n") ;

    node_->publish("left_cb_debug", cb_helper_.getLeftDebug()) ;
    node_->publish("right_cb_debug", cb_helper_.getRightDebug()) ;
  }

  void msgTimeout(ros::Time t)
  {
    ROS_WARN("%f - Timeout", t.toSec()) ;
    if (left_image_msg_.header.stamp != t)
      printf("- left_image  %f\n", left_image_msg_.header.stamp.toSec()) ;
    else
      printf("+ left_image  %f\n", left_image_msg_.header.stamp.toSec()) ;

    if (left_info_msg_.header.stamp != t)
      printf("- left info   %f\n", left_info_msg_.header.stamp.toSec()) ;
    else
      printf("+ left info   %f\n", left_info_msg_.header.stamp.toSec()) ;

    if (right_image_msg_.header.stamp != t)
      printf("- right image %f\n", right_image_msg_.header.stamp.toSec()) ;
    else
      printf("+ right image %f\n", right_image_msg_.header.stamp.toSec()) ;

    if (right_info_msg_.header.stamp != t)
      printf("- right info  %f\n", right_info_msg_.header.stamp.toSec()) ;
    else
      printf("+ right info  %f\n", right_info_msg_.header.stamp.toSec()) ;
  }

private :
  ros::Node* node_ ;
  StereoCheckerboardHelper cb_helper_ ;
  TopicSynchronizer<StereoCheckerboardNode> sync_ ;

  image_msgs::Image left_image_msg_ ;
  image_msgs::CamInfo left_info_msg_ ;
  image_msgs::Image right_image_msg_ ;
  image_msgs::CamInfo right_info_msg_ ;
} ;



int main(int argc, char** argv)
{
  ros::init(argc, argv) ;

  ros::Node node("stereo_cb_node") ;

  StereoCheckerboardNode cb(&node) ;

  node.spin() ;

}
