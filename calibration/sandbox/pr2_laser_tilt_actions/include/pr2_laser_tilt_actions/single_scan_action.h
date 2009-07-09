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

#ifndef PR2_CALIBRATION_ACTIONS_SINGLE_SCAN_ACTION_H_
#define PR2_CALIBRATION_ACTIONS_SINGLE_SCAN_ACTION_H_

#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "ros/ros.h"
#include "robot_actions/action.h"

#include "pr2_msgs/LaserScannerSignal.h"
#include "pr2_laser_tilt_actions/SingleScanCmd.h"
#include "pr2_laser_tilt_actions/Interval.h"

namespace pr2_laser_tilt_actions
{

/**
 * \brief Commands the hokuyo to move through a single sweep
 * Given a start angle, stop angle, and duration, this action will command the hokuyo
 * to move through this sweep.  It this returns the start time and stop time as action
 * feedback, which can be used to extract data from any type of laser assembler
 * (ie PointCloudAssembler).
 */
class SingleScanAction : public robot_actions::Action<SingleScanCmd, Interval>
{
public:
  SingleScanAction();

  virtual robot_actions::ResultStatus execute(const SingleScanCmd& goal, Interval& feedback) ;

private:
  void signalCallback(const pr2_msgs::LaserScannerSignalConstPtr& signal) ;

  ros::NodeHandle n_ ;
  ros::Subscriber sub_ ;

  std::string controller_;

  boost::mutex signal_mutex_ ;
  boost::condition signal_available_ ;
  pr2_msgs::LaserScannerSignal signal_ ;

} ;

}


#endif
