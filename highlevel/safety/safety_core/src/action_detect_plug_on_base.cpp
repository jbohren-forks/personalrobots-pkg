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


#include <safety_core/action_detect_plug_on_base.h>
#include <math.h>

using namespace safety_core;

DetectPlugOnBaseAction::DetectPlugOnBaseAction(ros::Node& node) :
  robot_actions::Action<std_msgs::Empty, robot_msgs::PlugStow>("detect_plug_on_base"),
  action_name_("detect_plug_on_base"),
  node_(node),
  request_preempt_(false),
  laser_controller_("laser_tilt_controller")
{
  detector_ = new PlugOnBaseDetector::PlugOnBaseDetector(node);
  detector_->deactivate();
  node_.subscribe ("~plug_stow_info", plug_stow_msg, &DetectPlugOnBaseAction::localizePlug, this, 1);
};

DetectPlugOnBaseAction::~DetectPlugOnBaseAction()
{
  if(detector_) delete detector_;
};

void DetectPlugOnBaseAction::handleActivate(const std_msgs::Empty& empty)
{
  notifyActivated();

  reset();
  node_.param(action_name_ + "/laser_tilt_controller", laser_controller_, laser_controller_);
  
  if(laser_controller_ == "")
  {
    ROS_ERROR("%s: tilt_laser_controller param was not set.",action_name_.c_str());
    plug_stow_.stowed = 0;
    notifyAborted(plug_stow_);
    return;
  }
 
  if (!ros::service::call(laser_controller_ + "/set_periodic_cmd", req_laser_, res_laser_))
  {
    if (!isActive())
    {
      return;
    }
    else
    {
      ROS_ERROR("%s: Failed to start laser.", action_name_.c_str());
      plug_stow_.stowed = 0;
      notifyAborted(plug_stow_);
    }
    return;
  }
 
  detector_->activate();

  return;
}

void DetectPlugOnBaseAction::handlePreempt()
{
  ROS_INFO("%s: preempted.", action_name_.c_str());
  plug_stow_.stowed = 0;
  notifyPreempted(plug_stow_);
  return;
}

void DetectPlugOnBaseAction::reset()
{
  not_found_count_ = 0;
  found_count_ = 0;

  std_x_ = 0;
  std_y_ = 0;
  std_z_ = 0;

  sum_x_ = 0;
  sum_y_ = 0;
  sum_z_ = 0;

  sum_sq_x_ = 0;
  sum_sq_y_ = 0;
  sum_sq_z_ = 0;

  plug_stow_.stowed = 0;
  plug_stow_.plug_centroid.x = 0;
  plug_stow_.plug_centroid.y = 0;
  plug_stow_.plug_centroid.z = 0;

  req_laser_.command.profile = "linear";
  req_laser_.command.period = 6;
  req_laser_.command.amplitude = 0.11;
  req_laser_.command.offset = 1.36;

}

void DetectPlugOnBaseAction::localizePlug()
{
  if(!isActive())
    return;

  if(plug_stow_msg.stowed == 0)
  {
    not_found_count_++;
    if(not_found_count_ > 10)
    {
      ROS_INFO("%s: aborted.", action_name_.c_str());
      plug_stow_.stowed = 0;
      notifyAborted(plug_stow_);
      return;
    }
    return;
  }
  else
  {
    found_count_++;
    // x
    sum_x_ += plug_stow_msg.plug_centroid.x;
    plug_stow_.plug_centroid.x = sum_x_ / found_count_;
    sum_sq_x_ += pow(plug_stow_msg.plug_centroid.x, 2);
    std_x_ = sqrt((sum_sq_x_/found_count_) - pow(plug_stow_.plug_centroid.x, 2));

    // y
    sum_y_ += plug_stow_msg.plug_centroid.y;
    plug_stow_.plug_centroid.y = sum_y_ / found_count_;
    sum_sq_y_ += pow(plug_stow_msg.plug_centroid.y, 2);
    std_y_ = sqrt((sum_sq_y_/found_count_) - pow(plug_stow_.plug_centroid.y, 2));

    // z
    sum_z_ += plug_stow_msg.plug_centroid.z;
    plug_stow_.plug_centroid.z = sum_z_ / found_count_;
    sum_sq_z_ += pow(plug_stow_msg.plug_centroid.z, 2);
    std_z_ = sqrt((sum_sq_z_/found_count_) - pow(plug_stow_.plug_centroid.z, 2));

    if(found_count_ > 3 && std_x_ < 0.05 && std_y_ < 0.05 && std_z_ < 0.05)
    {
      ROS_INFO("%s: succeeded.", action_name_.c_str());
      plug_stow_.header.frame_id = plug_stow_msg.header.frame_id;    
      plug_stow_.stowed = 1;
      detector_->deactivate();
      notifySucceeded(plug_stow_);
      
    }
  }

  return;
}
