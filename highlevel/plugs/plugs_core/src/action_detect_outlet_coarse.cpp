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

// Srvs
#include <outlet_detection/OutletDetection.h>

#include <plugs_core/action_detect_outlet_coarse.h>

namespace plugs_core
{

DetectOutletCoarseAction::DetectOutletCoarseAction()
				: robot_actions::Action<robot_msgs::PointStamped, robot_msgs::PoseStamped>("detect_outlet_coarse"),
          node_(ros::Node::instance()),
          action_name_("detect_outlet_coarse"),
          head_controller_("head_controller")
{
  node_->param(action_name_ + "/head_controller", head_controller_, head_controller_);

  if(head_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, head controller param was not set.", action_name_.c_str());
      terminate();
      return;
    }

	node_->advertise<robot_msgs::PointStamped>(head_controller_ + "/head_track_point",10);
}

DetectOutletCoarseAction::~DetectOutletCoarseAction()
{
	node_->unadvertise(head_controller_ + "/head_track_point");
}


robot_actions::ResultStatus DetectOutletCoarseAction::execute(const robot_msgs::PointStamped& point, robot_msgs::PoseStamped& feedback){


  ROS_DEBUG("%s: executing.", action_name_.c_str());

	
	if (!spotOutlet(point, feedback)){
		if (isPreemptRequested()){
			ROS_INFO("DetectOutletCoarseAction: Preempted");
			return robot_actions::PREEMPTED;
		}
		else{
			ROS_INFO("DetectOutletCoarseAction: Aborted");
			return robot_actions::ABORTED;
		}
	}
	else{
          ROS_DEBUG("%s: succeeded.", action_name_.c_str());
		return robot_actions::SUCCESS;
	}


}


bool DetectOutletCoarseAction::spotOutlet(const robot_msgs::PointStamped& outlet_estimate, robot_msgs::PoseStamped& pose)
{
	// turn head to face outlet
	node_->publish(head_controller_ + "/head_track_point", outlet_estimate);

	outlet_detection::OutletDetection::Request req;
	outlet_detection::OutletDetection::Response resp;

	req.point = outlet_estimate;

	if (ros::service::call("outlet_spotting/coarse_outlet_detect", req, resp)) {
		pose = resp.pose;

        robot_msgs::PointStamped outlet_final_position;
        outlet_final_position.header.frame_id = pose.header.frame_id;
        outlet_final_position.header.stamp = pose.header.stamp;
        outlet_final_position.point = pose.pose.position;

        node_->publish(head_controller_ + "/head_track_point", outlet_final_position);
		return true;
	}
	else {
		return false;
	}
}
}
