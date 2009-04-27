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
 *   * Neither the name of Willow Garage nor the names of its
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


#include <plugs_core/action_detect_outlet_fine.h>

namespace plugs_core
{

DetectOutletFineAction::DetectOutletFineAction(ros::Node& node)
  : robot_actions::Action<robot_msgs::PointStamped, robot_msgs::PoseStamped>("detect_outlet_fine"),
    node_(node)
{
  node_.setParam("~display", "false");
  detector_ = new OutletTracker::OutletTracker(node);
  detector_->deactivate();  
  node_.subscribe("~outlet_pose", outlet_pose_msg_, &DetectOutletFineAction::foundOutlet, this, 1);

}

DetectOutletFineAction::~DetectOutletFineAction()
{
  if(detector_) delete detector_;
};

robot_actions::ResultStatus DetectOutletFineAction::execute(const robot_msgs::PointStamped& point, robot_msgs::PoseStamped& feedback){

  detector_->activate();
  return waitForDeactivation(feedback);

}

void DetectOutletFineAction::foundOutlet()
{
  if (!isActive())
    return;

  if (isPreemptRequested())
    {
      deactivate(robot_actions::PREEMPTED, outlet_pose_msg_);
      return;
    }
  
  deactivate(robot_actions::SUCCESS, outlet_pose_msg_);
  detector_->deactivate();
  
}

}
