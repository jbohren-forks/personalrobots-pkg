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


#include "stereo_checkerboard_detector/mono_checkerboard_helper.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;

bool MonoCheckerboardHelper::getCorners(image_msgs::Image& image, vector<robot_msgs::Point>& ros_corners)
{
  // Always process checkerboard images as mono images
  if (!img_bridge_.fromImage(image, "mono"))
  {
    ROS_ERROR("Error opening image") ;
    return false ;
  }

  // Convert the ROS Image into openCV's IPL image
  IplImage* cv_image = img_bridge_.toIpl() ;

  vector<CvPoint2D32f> cv_corners ;

  bool found = getCorners(cv_image, cv_corners) ;
  ros_corners.resize( cv_corners.size() ) ;

  for (unsigned int i=0; i<cv_corners.size(); i++)
  {
    ros_corners[i].x = cv_corners[i].x ;
    ros_corners[i].y = cv_corners[i].y ;
    ros_corners[i].z = 0.0 ;
  }

  return found ;
}

bool MonoCheckerboardHelper::getCorners(const IplImage* image, vector<CvPoint2D32f>& corners)
{
  corners.clear() ;       // Make sure there's no junk in our output vector
  corners.resize(board_size_.width * board_size_.height) ;

  int num_corners ;
  int cb_found = cvFindChessboardCorners( image, board_size_, &corners[0], &num_corners,
                                          CV_CALIB_CB_ADAPTIVE_THRESH ) ;

  //! \todo Add border threshold code here. Should look similar to Rosen's cb detector's threshold code

  // Process more, only if we found a checkerboard
  if (cb_found)
  {
    // Subpixel fine-tuning stuff
    cvFindCornerSubPix(image, &corners[0], num_corners,
                       search_win_size_,
                       cvSize(2,2),
                       cvTermCriteria(CV_TERMCRIT_ITER,20,1e-2)) ;
    return true ;
  }
  return false ;
}
