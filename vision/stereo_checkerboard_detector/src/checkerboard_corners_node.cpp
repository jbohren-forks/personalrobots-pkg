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

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "robot_msgs/PointCloud.h"
//#include "stereo_checkerboard_detector/mono_checkerboard_helper.h"
#include "opencv_latest/CvBridge.h"

//using namespace stereo_checkerboard_detector ;
using namespace std ;

class CheckerboardCornersNode
{

public:
  CheckerboardCornersNode()
  {
    //! \todo make checkerboard size a parameter
    setSize(6,8) ;

    corners_pub_ = nh_.advertise<robot_msgs::PointCloud>("corners", 10) ;
    debug_pub_   = nh_.advertise<sensor_msgs::Image>("~debug_image", 1) ;

    image_sub_   = nh_.subscribe("image", 1, &CheckerboardCornersNode::imageCallback, this) ;

    debug_on_ = true ;
  }

  void setSize(int w, int h)
  {
    board_size_ = cvSize(w,h) ;
    // helper_.setSize(w,h) ;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // Always process checkerboard images as mono images
    if (!img_bridge_.fromImage(*msg, "mono"))
    {
      ROS_ERROR("Error opening image") ;
      return ;
    }

    // Convert the ROS Image into openCV's IPL image
    IplImage* cv_image = img_bridge_.toIpl() ;
    const int scale = 2 ;
    IplImage* cv_image_scaled = cvCreateImage(cvSize(cv_image->width*scale, cv_image->height*scale),
                                              cv_image->depth, cv_image->nChannels) ;
    cvResize(cv_image, cv_image_scaled, CV_INTER_LINEAR);

    vector<CvPoint2D32f> cv_corners ;
    cv_corners.resize(board_size_.width*board_size_.height) ;
    // ******** Checkerboard Extraction **********
    //bool found = helper_.getCorners(cv_image, cv_corners) ;
    int num_corners ;
    int found = cvFindChessboardCorners( cv_image_scaled, board_size_, &cv_corners[0], &num_corners,
                                         CV_CALIB_CB_ADAPTIVE_THRESH) ;

    if(found)
    {
      // Subpixel fine-tuning stuff
      cvFindCornerSubPix(cv_image_scaled, &cv_corners[0], num_corners,
                         cvSize(2,2),
                         cvSize(-1,-1),
                         cvTermCriteria(CV_TERMCRIT_ITER,20,1e-2)) ;
    }

    // Downscale the pixel locations
    for (unsigned int i=0; i<cv_corners.size(); i++)
    {
      cv_corners[i].x /= scale ;
      cv_corners[i].y /= scale ;
    }

    robot_msgs::PointCloud cloud ;
    cloud.header.frame_id = msg->header.frame_id ;
    cloud.header.stamp = msg->header.stamp ;
    if (found)
    {
      cloud.pts.resize( cv_corners.size() ) ;

      for (unsigned int i=0; i<cv_corners.size(); i++)
      {
        cloud.pts[i].x = cv_corners[i].x ;
        cloud.pts[i].y = cv_corners[i].y ;
        cloud.pts[i].z = 1.0 ;
      }
    }
    else
      cloud.pts.resize(0) ;

    corners_pub_.publish(cloud) ;

    if (debug_on_)
    {
      IplImage* cv_debug = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 3) ;
      cvCvtColor(cv_image, cv_debug, CV_GRAY2BGR) ;
      //if (found)
      //  cvDrawChessboardCorners(cv_debug, board_size_, &cv_corners[0], cv_corners.size(), found) ;

      if (found)
      {
        for (unsigned int i=0; i<cv_corners.size(); i++)
        {
          cvCircle(cv_debug, cvPoint(cv_corners[i].x, cv_corners[i].y), 2, cvScalar(0,255,0), 1) ;
        }
      }
      else
      {
        for (unsigned int i=0; i<cv_corners.size(); i++)
        {
          cvCircle(cv_debug, cvPoint(cv_corners[i].x, cv_corners[i].y), 2, cvScalar(255,0,0), 1) ;
        }
      }

      img_bridge_.fromIpltoRosImage(cv_debug, debug_image_) ;
      debug_image_.header.stamp = msg->header.stamp ;
      debug_pub_.publish(debug_image_) ;
    }
  }


private:
  ros::NodeHandle nh_ ;
  //MonoCheckerboardHelper helper_ ;
  ros::Subscriber image_sub_ ;
  ros::Publisher corners_pub_ ;
  ros::Publisher debug_pub_ ;

  sensor_msgs::CvBridge img_bridge_ ;

  bool debug_on_ ;
  CvSize board_size_ ;
  sensor_msgs::Image debug_image_ ;
} ;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "checkerboard_corners_node") ;

  CheckerboardCornersNode cb_corners ;

  ros::spin() ;

}
