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

#include "stereo_checkerboard_detector/stereo_checkerboard_helper.h"

using namespace std ;
using namespace stereo_checkerboard_detector ;


bool StereoCheckerboardHelper::findCheckerboard(sensor_msgs::Image& left, sensor_msgs::Image& right,
                                                const sensor_msgs::CameraInfo& left_info, const sensor_msgs::CameraInfo& right_info)
{
  if (!left_bridge_.fromImage(left, "mono") ||
      !right_bridge_.fromImage(right, "mono") )
  {
    ROS_ERROR("Error opening image") ;
    return false ;
  }

  vector<CvPoint2D32f> left_corners ;
  vector<CvPoint2D32f> right_corners ;

  bool left_found  =  left_helper_.getCorners(left_bridge_.toIpl(), left_corners) ;
  bool right_found = right_helper_.getCorners(right_bridge_.toIpl(), right_corners) ;

  left_xy_.resize(left_corners.size()) ;
  for (unsigned int i=0; i<left_corners.size(); i++)
  {
    left_xy_[i].x = left_corners[i].x ;
    left_xy_[i].y = left_corners[i].y ;
    left_xy_[i].z = 0.0 ;
  }

  right_xy_.resize(right_corners.size()) ;
  for (unsigned int i=0; i<right_corners.size(); i++)
  {
    right_xy_[i].x = right_corners[i].x ;
    right_xy_[i].y = right_corners[i].y ;
    right_xy_[i].z = 0.0 ;
  }

  // Build Debug Images
  IplImage* left_debug = cvCreateImage(cvGetSize(left_bridge_.toIpl()), IPL_DEPTH_8U, 3) ;
  IplImage* right_debug = cvCreateImage(cvGetSize(right_bridge_.toIpl()), IPL_DEPTH_8U, 3) ;

  cvCvtColor(left_bridge_.toIpl(), left_debug, CV_GRAY2BGR) ;
  cvCvtColor(right_bridge_.toIpl(), right_debug, CV_GRAY2BGR) ;

  if (left_found)
    cvDrawChessboardCorners(left_debug, board_size_, &left_corners[0], left_corners.size(), left_found) ;
  if (right_found)
    cvDrawChessboardCorners(right_debug, board_size_, &right_corners[0], right_corners.size(), right_found) ;

  left_bridge_.fromIpltoRosImage(left_debug, left_ros_debug_) ;
  right_bridge_.fromIpltoRosImage(right_debug, right_ros_debug_) ;
  left_ros_debug_.header.stamp = left.header.stamp ;
  right_ros_debug_.header.stamp = right.header.stamp ;

  cvReleaseImage(&left_debug) ;
  cvReleaseImage(&right_debug) ;

  if (left_found && right_found)
  {
    CvMat* uvd = cvCreateMat(left_corners.size(), 1, CV_32FC3) ;        //!< Stores disparities
    CvMat* xyz = cvCreateMat(left_corners.size(), 1, CV_32FC3) ;        //!< Stores disparities

    // Compute disparities
    reproj_helper_.computeDisparity(left_corners, right_corners, uvd) ;

    printf(" Left: (%.2f, %.2f)\n", left_corners[0].x, left_corners[0].y) ;
    printf("Right: (%.2f, %.2f)\n", right_corners[0].x, right_corners[0].y) ;
    printf(" Disp: %.2f\n", *((float*) CV_MAT_ELEM_PTR(*uvd, 0, 0)+2) ) ;

    // Using Projection Matricies, convert disparities into cartesian locations
    reproj_helper_.reproject(uvd, left_info, right_info, xyz) ;

    xyz_.resize(left_corners.size()) ;
    for (unsigned int i=0; i<left_corners.size(); i++)
    {
      xyz_[i].x = *((float*) CV_MAT_ELEM_PTR(*xyz, i, 0)+0) ;
      xyz_[i].y = *((float*) CV_MAT_ELEM_PTR(*xyz, i, 0)+1) ;
      xyz_[i].z = *((float*) CV_MAT_ELEM_PTR(*xyz, i, 0)+2) ;
    }

    // Recast xyz points into a better shape
    CvMat xyz_Nx3 = cvMat(left_corners.size(), 3, CV_32FC1, CV_MAT_ELEM_PTR( *xyz, 0, 0) ) ;
    pose_helper_.getPose(&xyz_Nx3, pose_) ;

    cvReleaseMat(&uvd) ;
    cvReleaseMat(&xyz) ;
    return true ;
  }

  return false ;
}
