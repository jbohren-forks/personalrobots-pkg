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

#include "sensor_msgs/PointCloud.h"

#include "topic_synchronizer/topic_synchronizer.h"
#include "visualization_msgs/Marker.h"

#include "calibration_msgs/CbStereoCorners.h"
#include "calibration_msgs/CbCamCorners.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;
using namespace calibration_msgs ;

class StereoCheckerboardNode
{

public:
  StereoCheckerboardNode(ros::Node* node) : node_(node),
  sync_(node_, this, &StereoCheckerboardNode::msgCallback,
        ros::Duration().fromSec(0.05),
        &StereoCheckerboardNode::msgTimeout )
  {
    num_x_ = 6 ;
    num_y_ = 8 ;
    cb_helper_.setSize(num_x_, num_y_, .107) ;

    sync_.subscribe("stereo/left/image_rect", left_image_msg_,  1) ;
    sync_.subscribe("stereo/left/cam_info",  left_info_msg_,   1) ;
    sync_.subscribe("stereo/right/image_rect",right_image_msg_, 1) ;
    sync_.subscribe("stereo/right/cam_info", right_info_msg_,  1) ;

    node_->advertise<calibration_msgs::CbStereoCorners>("cb_stereo_corners", 1) ;
    node_->advertise<sensor_msgs::PointCloud>("cb_corners", 1) ;
    node_->advertise<sensor_msgs::PointCloud>("cb_corners_left", 1) ;
    node_->advertise<sensor_msgs::PointCloud>("cb_corners_right", 1) ;
    node_->advertise<geometry_msgs::PoseStamped>("cb_pose", 1) ;
    node_->advertise<sensor_msgs::Image>("left_cb_debug",1) ;
    node_->advertise<sensor_msgs::Image>("right_cb_debug",1) ;
    node->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    sync_.ready() ;
  }

  ~StereoCheckerboardNode()
  {

  }

  void pointVecToCloud(const vector<geometry_msgs::Point>& vec, sensor_msgs::PointCloud& cloud)
  {
    cloud.pts.resize(vec.size()) ;
    for (unsigned int i=0; i<vec.size(); i++)
    {
      cloud.pts[i].x = vec[i].x ;
      cloud.pts[i].y = vec[i].y ;
      cloud.pts[i].z = vec[i].z ;
    }
  }

  void pointVecToCbCamCorners(const vector<geometry_msgs::Point>& vec, unsigned int num_x, unsigned int num_y,
                              calibration_msgs::CbCamCorners& corners)
  {
    corners.num_x = num_x ;
    corners.num_y = num_y ;
    if (vec.size() != num_x * num_y)
    {
      ROS_ERROR("vec.size()=%u, num_x=%u, num_y=%u", vec.size(), num_x, num_y) ;
      return ;
    }

    for (unsigned int j=0; j<num_y; j++)
    {
      for (unsigned int i=0; i<num_x; i++)
      {
        const unsigned int ind = j*num_x + num_x ;
        corners.corners[ind].x_index = i ;
        corners.corners[ind].y_index = j ;
        corners.corners[ind].point.x = vec[ind].x ;
        corners.corners[ind].point.y = vec[ind].y ;
      }
    }
  }

  void msgCallback(ros::Time t)
  {
    bool found = cb_helper_.findCheckerboard(left_image_msg_, right_image_msg_, left_info_msg_, right_info_msg_) ;
    // Define the cloud we'll be using to publish with
    sensor_msgs::PointCloud cloud ;
    cloud.header.frame_id = left_image_msg_.header.frame_id ;
    cloud.header.stamp = left_image_msg_.header.stamp ;

    CbStereoCorners stereo_corners ;
    stereo_corners.header.frame_id = left_image_msg_.header.frame_id ;
    stereo_corners.header.stamp    = left_image_msg_.header.stamp ;
    stereo_corners.left.header  = stereo_corners.header ;
    stereo_corners.right.header = stereo_corners.header ;

    if (found)
    {
      //unsigned int N = xyz.size() ;


      // Publish the XYZ corners
      vector<geometry_msgs::Point> point_vec ;
      cb_helper_.getCorners(point_vec) ;
      pointVecToCloud(point_vec, cloud) ;
      node_->publish("cb_corners", cloud) ;

      // Publish the left corners
      vector<geometry_msgs::Point> left_point_vec ;
      cb_helper_.getCornersLeft(left_point_vec) ;
      pointVecToCloud(left_point_vec, cloud) ;
      pointVecToCbCamCorners(left_point_vec, num_x_, num_y_, stereo_corners.left) ;
      node_->publish("cb_corners_left", cloud) ;

      // Publish the right corners
      vector<geometry_msgs::Point> right_point_vec ;
      cb_helper_.getCornersRight(right_point_vec) ;
      pointVecToCloud(right_point_vec, cloud) ;
      pointVecToCbCamCorners(right_point_vec, num_x_, num_y_, stereo_corners.right) ;
      node_->publish("cb_corners_right", cloud) ;

      node_->publish("cb_stereo_corners", stereo_corners) ;

      // Publish a mini-axes defining the checkerboard
      tf::Pose pose ;
      cb_helper_.getPose(pose) ;
      publishMarker(pose) ;

      geometry_msgs::PoseStamped pose_stamped_msg ;
      tf::poseTFToMsg(pose, pose_stamped_msg.pose) ;
      pose_stamped_msg.header.frame_id = left_image_msg_.header.frame_id ;
      pose_stamped_msg.header.stamp = left_image_msg_.header.stamp ;
      node_->publish("cb_pose", pose_stamped_msg) ;

      printf("Found CB!\n") ;
    }
    else
    {
      cloud.pts.clear() ;
      node_->publish("cb_corners_left", cloud) ;
      node_->publish("cb_corners_right", cloud) ;

      stereo_corners.left.num_x = num_x_ ;
      stereo_corners.left.num_y = num_y_ ;
      stereo_corners.left.corners.clear() ;
      stereo_corners.right.num_x = num_x_ ;
      stereo_corners.right.num_y = num_y_ ;
      stereo_corners.left.corners.clear() ;
      node_->publish("cb_stereo_corners", stereo_corners) ;
      printf("Not found\n") ;
    }

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

  void publishMarker(const tf::Pose& pose)
  {
    visualization_msgs::Marker marker ;
    marker.header.frame_id = left_image_msg_.header.frame_id ;
    marker.header.stamp = left_image_msg_.header.stamp ;
    marker.ns = "cb_detector" ;               // Should we querying our name from the node?
    marker.id = 0 ;
    marker.type = visualization_msgs::Marker::ARROW ;
    marker.action = visualization_msgs::Marker::ADD ;
    tf::poseTFToMsg(pose, marker.pose) ;
    marker.scale.x = .2 ;
    marker.scale.y = .2 ;
    marker.scale.z = .2 ;
    marker.color.r = 1.0 ;
    marker.color.g = 0.0 ;
    marker.color.b = 0.0 ;
    marker.color.a = 1.0 ;
    marker.lifetime = ros::Duration().fromSec(0.0) ;
    node_->publish("visualization_marker", marker) ;

    btQuaternion x_to_y(btVector3(0.0, 0.0, 1.0), M_PI/2) ;
    btQuaternion x_to_z(btVector3(0.0,-1.0, 0.0), M_PI/2) ;

    tf::poseTFToMsg( pose*btTransform(x_to_y), marker.pose) ;
    marker.id = 1 ;
    marker.color.r = 0.0 ;
    marker.color.g = 1.0 ;
    marker.color.b = 0.0 ;
    marker.color.a = 1.0 ;
    node_->publish("visualization_marker", marker) ;


    tf::poseTFToMsg( pose*btTransform(x_to_z), marker.pose) ;
    marker.id = 2 ;
    marker.color.r = 0.0 ;
    marker.color.g = 0.0 ;
    marker.color.b = 1.0 ;
    marker.color.a = 1.0 ;
    node_->publish("visualization_marker", marker) ;


  }

private :
  ros::Node* node_ ;
  StereoCheckerboardHelper cb_helper_ ;
  TopicSynchronizer<StereoCheckerboardNode> sync_ ;

  unsigned int num_x_ ;
  unsigned int num_y_ ;

  sensor_msgs::Image left_image_msg_ ;
  sensor_msgs::CamInfo left_info_msg_ ;
  sensor_msgs::Image right_image_msg_ ;
  sensor_msgs::CamInfo right_info_msg_ ;
} ;



int main(int argc, char** argv)
{
  ros::init(argc, argv) ;

  ros::Node node("stereo_cb_node") ;

  StereoCheckerboardNode cb(&node) ;

  node.spin() ;

}
