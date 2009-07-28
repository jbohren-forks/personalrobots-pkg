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

#include <vector>

#include "ros/node.h"

#include "opencv/cv.h"
#include "opencv_latest/CvBridge.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/CamInfo.h"
#include "topic_synchronizer/topic_synchronizer.h"

using namespace std ;

/**
 * A simple test node that can be used to overlay point clouds onto images
 */
class ImagePcDebugger
{
public:
  ImagePcDebugger(ros::Node* node) : node_(node),
  sync_(node_, this, &ImagePcDebugger::msgCallback,
        ros::Duration().fromSec(0.5),
        &ImagePcDebugger::msgTimeout )
  {
    sync_.subscribe("stereo/left/image_rect", image_msg_,  1) ;
    sync_.subscribe("stereo/left/cam_info",  info_msg_,   1) ;
    sync_.subscribe("cb_corners",pc_msg_, 1) ;

    node_->advertise<sensor_msgs::Image>("pc_debug",1) ;

    sync_.ready() ;
  }

  void msgCallback(ros::Time t)
  {
    printf("Callback!\n") ;
    const double* P = &info_msg_.P[0] ;

    vector<CvPoint> cv_pts ;
    cv_pts.resize(pc_msg_.pts.size()) ;

    // Project 3D points onto camera plane (assume that frame_ids match)
    for (unsigned int i=0; i<pc_msg_.pts.size(); i++)
    {
      const geometry_msgs::Point32& pt = pc_msg_.pts[i] ;
      double u = P[0]*pt.x + P[1]*pt.y + P[2]*pt.z + P[3] ;
      double v = P[4]*pt.x + P[5]*pt.y + P[6]*pt.z + P[7] ;
      double w = P[8]*pt.x + P[9]*pt.y + P[10]*pt.z + P[11] ;
      cv_pts[i] = cvPoint(u/w, v/w) ;
    }

    if (!img_bridge_.fromImage(image_msg_, "mono"))
    {
      ROS_ERROR("Error opening image") ;
      return ;
    }

    IplImage* cv_image = img_bridge_.toIpl() ;
    // Allocate an image to write debug data onto
    IplImage* cv_debug = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 3) ;
    cvCvtColor(cv_image, cv_debug, CV_GRAY2BGR) ;

    // Draw on the debug image
    for (unsigned int i=0; i<cv_pts.size(); i++)
      cvCircle(cv_debug, cv_pts[i], 5, cvScalar(0,255,0), 1) ;

    // Convert openCV image into a ROS Message
    sensor_msgs::Image ros_debug_ ;
    img_bridge_.fromIpltoRosImage(cv_debug, ros_debug_) ;
    node_->publish("pc_debug", ros_debug_) ;

    cvReleaseImage(&cv_debug) ;
  }

  void msgTimeout(ros::Time t)
  {
    ROS_WARN("%f - Timeout", t.toSec()) ;
    if (image_msg_.header.stamp != t)
      printf("- left_image\n") ;
    else
      printf("+ left_image\n") ;

    if (info_msg_.header.stamp != t)
      printf("- left info\n") ;
    else
      printf("+ left info\n") ;

    if (pc_msg_.header.stamp != t)
      printf("- cloud\n") ;
    else
      printf("+ cloud\n") ;

  }

private:
  ros::Node* node_ ;
  TopicSynchronizer<ImagePcDebugger> sync_ ;
  sensor_msgs::CvBridge img_bridge_ ;

  sensor_msgs::Image image_msg_ ;
  sensor_msgs::CamInfo info_msg_ ;
  sensor_msgs::PointCloud pc_msg_ ;




} ;


int main(int argc, char** argv)
{
  ros::init(argc, argv) ;

  ros::Node node("pc_debug_node") ;

  ImagePcDebugger pc_test(&node) ;

  node.spin() ;

}

