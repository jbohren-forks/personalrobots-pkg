/*
 * Copyright (C) 2008, Jason Wolfe and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <map>

#include <ros/ros.h>

#include <planning_environment/monitors/planning_monitor.h>

namespace fk_node {


int main(int argc, char **argv)
{
	ros::init(argc, argv, "fk_node");
	ros::NodeHandle nh;

	tf::TransformListener                  tf;
	planning_environment::CollisionModels *collisionModels = new planning_environment::CollisionModels("robot_description");
	planning_environment::PlanningMonitor *planningMonitor = new planning_environment::PlanningMonitor(collisionModels, &tf);

	if (collisionModels_->loadedModels()) {
	    planningMonitor_->getEnvironmentModel()->setVerbose(true);
	    planningMonitor_->waitForState();
	    planningMonitor_->waitForMap();

		std::vector<std::string> arm_joint_names;
	    if(getControlJointNames(arm_joint_names)) {

	    	return 0;
	    }
	}

	ROS_ERROR("fk_node is invalid; quitting");
	return 1;
}


};
