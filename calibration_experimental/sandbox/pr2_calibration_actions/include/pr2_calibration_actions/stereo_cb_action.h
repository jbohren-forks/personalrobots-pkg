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

#ifndef PR2_CALIBRATION_ACTIONS_STEREO_CB_ACTION_H_
#define PR2_CALIBRATION_ACTIONS_STEREO_CB_ACTION_H_

#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "ros/ros.h"
#include "robot_actions/action.h"

#include "pr2_calibration_actions/CbCamCmd.h"
#include "calibration_msgs/CbStereoCorners.h"

namespace pr2_calibration_actions
{

/**
 *
 */
class StereoCbAction : public robot_actions::Action<CbCamCmd, calibration_msgs::CbStereoCorners>
{
public:
  StereoCbAction();

  virtual robot_actions::ResultStatus execute(const CbCamCmd& goal, calibration_msgs::CbStereoCorners& feedback) ;


private:
  void handlePreempt() ;
  void cornersCallback(const calibration_msgs::CbStereoCornersConstPtr& msg) ;
  void resetStereoAvg(const CbCamCmd& goal) ;
  bool pixelTooFar(const calibration_msgs::PixelPoint& a, const calibration_msgs::PixelPoint& b, double tol) ;

  ros::NodeHandle n_ ;
  ros::Subscriber sub_ ;

  std::string controller_;

  boost::mutex data_mutex_ ;
  boost::condition corners_available_ ;
  calibration_msgs::CbStereoCorners avg_ ;
  CbCamCmd goal_ ;
  unsigned int num_samples_ ;
  bool accumulating_ ;
  bool need_to_abort_ ;

} ;

}


#endif
