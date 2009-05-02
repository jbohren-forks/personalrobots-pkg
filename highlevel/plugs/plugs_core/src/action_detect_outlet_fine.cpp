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
    node_(node),
    action_name_("detect_outlet_fine"),
    head_controller_("head_controller"),
    detector_(NULL)    
{
  node_.param(action_name_ + "/head_controller", head_controller_, head_controller_);

  if(head_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, head controller param was not set.", action_name_.c_str());
      terminate();
      return;
    }
  node_.advertise<robot_msgs::PointStamped>(head_controller_ + "/head_track_point",10);
  node_.setParam("~display", 0);
  detector_ = new OutletTracker::OutletTracker(node);
  detector_->deactivate();  
  node_.subscribe("~outlet_pose", outlet_pose_msg_, &DetectOutletFineAction::foundOutlet, this, 1);

}

DetectOutletFineAction::~DetectOutletFineAction()
{
  if(detector_) delete detector_;
  node_.unadvertise(head_controller_ + "/head_track_point");
};

robot_actions::ResultStatus DetectOutletFineAction::execute(const robot_msgs::PointStamped& outlet_estimate, robot_msgs::PoseStamped& feedback)
{
  ROS_DEBUG("%s: executing.", action_name_.c_str());
  // point the head at the outlet
  node_.publish(head_controller_ + "/head_track_point", outlet_estimate);

  detector_->activate();
  return waitForDeactivation(feedback);

}

void DetectOutletFineAction::foundOutlet()
{
  if (!isActive())
    return;

  if (isPreemptRequested())
    {
      ROS_DEBUG("%s: preempted.", action_name_.c_str());
      deactivate(robot_actions::PREEMPTED, outlet_pose_msg_);
      return;
    }
  ROS_DEBUG("%s: succeeded.", action_name_.c_str());
  deactivate(robot_actions::SUCCESS, outlet_pose_msg_);
  detector_->deactivate();
  
}

}
