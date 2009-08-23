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

#include "robot_actions/action_client.h"

#include "pr2_laser_tilt_actions/Interval.h"
#include "pr2_laser_tilt_actions/SingleScanCmd.h"
#include "pr2_laser_tilt_actions/SingleScanActionState.h"


using namespace pr2_laser_tilt_actions ;

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_single_scan") ;

  ros::NodeHandle n ;

  boost::thread spinthread = boost::thread(boost::bind(&spinThread)) ;

  robot_actions::ActionClient<SingleScanCmd, SingleScanActionState, Interval> client("single_scan_action") ;
  ros::Duration duration ;

  duration.fromSec(3.0) ;
  ROS_DEBUG("Sleeping") ;
  duration.sleep() ;
  ROS_DEBUG("Done Sleeping") ;

  SingleScanCmd goal ;
  goal.start_angle = -.5 ;
  goal.end_angle = .5 ;
  goal.scan_duration = 5 ;

  Interval feedback ;

  robot_actions::ResultStatus result = client.execute(goal, feedback, ros::Duration().fromSec(10.0)) ;

  ROS_DEBUG("ResultStatus=%u", result) ;

  switch(result)
  {
    case robot_actions::SUCCESS :
      ROS_DEBUG("SUCCESS!") ;
      break ;
    case robot_actions::ABORTED :
      ROS_DEBUG("aborted") ;
      break ;
    case robot_actions::PREEMPTED :
      ROS_DEBUG("preempted") ;
      break ;
    default :
      ROS_DEBUG("other") ;
      break ;
  }

  ROS_DEBUG("Start Time: %f", feedback.start.toSec()) ;
  ROS_DEBUG("End   Time: %f", feedback.end.toSec()) ;

  return 0 ;
}
