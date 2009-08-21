/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include "pr2_calibration_actions/stereo_cb_action.h"
#include "calibration_msgs/CbCamCorners.h"

using namespace pr2_calibration_actions ;
using namespace calibration_msgs ;

StereoCbAction::StereoCbAction() : robot_actions::Action<CbCamCmd, CbStereoCorners>("stereo_cb_action")
{
  sub_ = n_.subscribe("cb_stereo_corners", 1, &StereoCbAction::cornersCallback, this) ;
  accumulating_ = false ;
}

robot_actions::ResultStatus StereoCbAction::execute(const CbCamCmd& goal, CbStereoCorners& feedback)
{
  boost::mutex::scoped_lock lock(data_mutex_) ;

  resetStereoAvg(goal) ;
  accumulating_ = true ;
  need_to_abort_ = false ;
  goal_ = goal ;

  boost::posix_time::milliseconds condition_timeout(1000.0f) ; // 1 sec signal timeout
  while(true)
  {
    corners_available_.timed_wait(data_mutex_, condition_timeout) ;

    if (need_to_abort_)
    {
      ROS_DEBUG("ABORTING") ;
      feedback = avg_ ;
      accumulating_ = false ;
      return robot_actions::ABORTED ;
    }

    if (isPreemptRequested())
    {
      ROS_DEBUG("PREEMPTED") ;
      feedback = avg_ ;
      accumulating_ = false ;
      return robot_actions::PREEMPTED ;
    }
  }
}

void StereoCbAction::handlePreempt()
{
  ROS_DEBUG("Handle Preempt") ;
  corners_available_.notify_all() ;
}

void StereoCbAction::cornersCallback(const calibration_msgs::CbStereoCornersConstPtr& msg)
{
  if (msg->left.corners.size() == 0)
  {
    ROS_INFO("Didn't see checkerboard in left cam") ;
    return ;
  }

  if (msg->right.corners.size() == 0)
  {
    ROS_INFO("Didn't see checkerboard in right cam") ;
    return ;
  }

  boost::mutex::scoped_lock lock(data_mutex_) ;

  if (!accumulating_)
    return ;

  const unsigned int N = msg->left.corners.size() ;
  if (msg->right.corners.size() != N)
  {
    ROS_ERROR("Left and right images don't have same # of corners") ;
    need_to_abort_ = true ;
    return ;
  }

  if (avg_.left.corners.size() != N)
  {
    ROS_ERROR("Received image does not have same # of corners as avg ") ;
    need_to_abort_ = true ;
    return ;
  }

  // Check pixel distances. Trigger an abort if the pixel dist is too far

  bool pixels_too_far = false ;
  if (num_samples_ > 0)
  {
    for (unsigned int i=0; i<N; i++)
    {
      pixels_too_far = pixels_too_far || pixelTooFar(avg_.left.corners[i].point,  msg->left.corners[i].point,  goal_.pixel_tol) ;
      pixels_too_far = pixels_too_far || pixelTooFar(avg_.right.corners[i].point, msg->right.corners[i].point, goal_.pixel_tol) ;
    }
  }

  if (pixels_too_far)
  {
    ROS_DEBUG("Some pixels were too far.  Need to abort") ;
    need_to_abort_ = true ;
  }

  for (unsigned int i=0; i<N; i++)
  {

    if (avg_.left.corners[i].x_index != msg->left.corners[i].x_index ||
        avg_.left.corners[i].y_index != msg->left.corners[i].y_index ||
        avg_.right.corners[i].x_index != msg->right.corners[i].x_index ||
        avg_.right.corners[i].y_index != msg->right.corners[i].y_index)
    {
      ROS_ERROR("Corner indicies don't line up. This action doesn't support this [tricky] case yet") ;
      need_to_abort_ = true ;
    }

    avg_.left.corners[i].point.x  = (avg_.left.corners[i].point.x*num_samples_  + msg->left.corners[i].point.x)/ (num_samples_+1) ;
    avg_.left.corners[i].point.y  = (avg_.left.corners[i].point.y*num_samples_  + msg->left.corners[i].point.y)/ (num_samples_+1) ;
    avg_.right.corners[i].point.x = (avg_.right.corners[i].point.x*num_samples_ + msg->right.corners[i].point.x)/(num_samples_+1) ;
    avg_.right.corners[i].point.y = (avg_.right.corners[i].point.y*num_samples_ + msg->right.corners[i].point.y)/(num_samples_+1) ;
  }
  num_samples_++ ;

  ROS_DEBUG("NumSamples: %u", num_samples_) ;

  corners_available_.notify_all() ;
}

void resetCbCamCorners(CbCamCorners& cam, const CbCamCmd& goal)
{
  cam.corners.resize(goal.num_x * goal.num_y) ;

  for (unsigned int j=0; j<goal.num_y; j++)
  {
    for (unsigned int i=0; i<goal.num_x; i++)
    {
      cam.corners[j*goal.num_x + i].x_index = i ;
      cam.corners[j*goal.num_x + i].y_index = j ;
      cam.corners[j*goal.num_x + i].point.x = 0.0 ;
      cam.corners[j*goal.num_x + i].point.y = 0.0 ;
    }
  }

  cam.num_x = goal.num_x ;
  cam.num_y = goal.num_y ;
}

void StereoCbAction::resetStereoAvg(const CbCamCmd& goal)
{
  num_samples_ = 0 ;
  resetCbCamCorners(avg_.left, goal) ;
  resetCbCamCorners(avg_.right, goal) ;
}


bool StereoCbAction::pixelTooFar(const PixelPoint& a, const PixelPoint& b, double tol)
{
  double dx = a.x - b.x ;
  double dy = a.y - b.y ;

  return dx > tol || dx < -tol || dy > tol || dy < -tol ;
}
