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

#include "pr2_laser_tilt_actions/single_scan_action.h"

#include "pr2_srvs/SetLaserTrajCmd.h"

using namespace std;
using namespace pr2_laser_tilt_actions;

SingleScanAction::SingleScanAction() : robot_actions::Action<SingleScanCmd, Interval>("single_scan_action")
{
  controller_ = "laser_tilt_controller" ;

  sub_ = n_.subscribe(controller_ + "/laser_scanner_signal", 1, &SingleScanAction::signalCallback, this) ;
}

robot_actions::ResultStatus SingleScanAction::execute(const SingleScanCmd& goal, Interval& feedback)
{
  pr2_srvs::SetLaserTrajCmd::Request req ;
  pr2_srvs::SetLaserTrajCmd::Response resp ;
  req.command.profile = "linear" ;
  req.command.pos.resize(3) ;
  req.command.time.resize(3) ;

  req.command.pos[0]  = goal.start_angle ;
  req.command.time[0] = 0.0 ;
  req.command.pos[1]  = goal.end_angle ;
  req.command.time[1] = goal.scan_duration ;
  req.command.pos[2]  = goal.start_angle ;
  req.command.time[2] = goal.scan_duration*2 ;

  if (!ros::service::call(controller_ + "/set_traj_cmd", req, resp))
  {
    ROS_ERROR("ABORTING - Error setting laser scanner trajectory command") ;
    return robot_actions::ABORTED ;
  }

  // Choose a trigger time that's comfortably after the start time, and comfortably before the end time
  ros::Time trigger_time = resp.start_time + ros::Duration().fromSec(goal.scan_duration*.5) ;


  ROS_DEBUG("Start Time: %f", resp.start_time.toSec()) ;
  ROS_DEBUG("Trigger Time: %f", trigger_time.toSec()) ;

  boost::posix_time::milliseconds condition_timeout(250.0f) ; // .25 sec signal timeout

  {
    boost::mutex::scoped_lock lock(signal_mutex_) ;
    while(true)
    {
      // Wait until we receive a LaserScannerSignal
      if (signal_available_.timed_wait(signal_mutex_, condition_timeout))
      {
        if (signal_.header.stamp > trigger_time)
        {
          ROS_DEBUG("Stamp is later than trigger time") ;
          feedback.start = resp.start_time ;
          feedback.end   = signal_.header.stamp ;
          return robot_actions::SUCCESS ;
        }
        else
          ROS_DEBUG("Stamp still too early. Waiting again...") ;
      }
      else
        ROS_DEBUG("Condition Looping") ;

      if (isPreemptRequested())
      {
        ROS_DEBUG("PREEMPTED") ;
        feedback.start = resp.start_time ;
        feedback.end = ros::Time() ;        // There's no end time, so set to zero
        return robot_actions::PREEMPTED ;
      }
    }
  }
}

void SingleScanAction::signalCallback(const pr2_msgs::LaserScannerSignalConstPtr& signal)
{
  ROS_DEBUG("signalCallback") ;
  boost::mutex::scoped_lock lock(signal_mutex_) ;
  signal_ = *signal ;

  // Tell the execute function that we just got a laserScannerSignal
  signal_available_.notify_all() ;
}
