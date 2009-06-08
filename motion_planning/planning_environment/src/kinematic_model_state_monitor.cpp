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

/** \author Ioan Sucan */

#include "planning_environment/kinematic_model_state_monitor.h"
#include <sstream>

void planning_environment::KinematicModelStateMonitor::setupRSM(void)
{
    pose_.setIdentity();
    kmodel_ = NULL;
    robotState_ = NULL;
    onStateUpdate_ = NULL;
    havePose_ = haveMechanismState_ = false;
    if (rm_->loadedModels())
    {
	kmodel_ = rm_->getKinematicModel().get();
	robotState_ = kmodel_->newStateParams();
	robotState_->setInRobotFrame();
	
	for (unsigned int i = 0 ; i < kmodel_->getModelInfo().floatingJoints.size() ; ++i)
	    floatingJoints_.push_back(kmodel_->getModelInfo().parameterName[kmodel_->getModelInfo().floatingJoints[i]]);
	for (unsigned int i = 0 ; i < kmodel_->getModelInfo().planarJoints.size() ; ++i)
	    planarJoints_.push_back(kmodel_->getModelInfo().parameterName[kmodel_->getModelInfo().planarJoints[i]]);
	
	mechanismStateSubscriber_ = nh_.subscribe("mechanism_state", 1, &KinematicModelStateMonitor::mechanismStateCallback, this);
	if (includePose_)
	    localizedPoseSubscriber_  = nh_.subscribe("localized_pose",  1, &KinematicModelStateMonitor::localizedPoseCallback,  this);
    }
}

void planning_environment::KinematicModelStateMonitor::mechanismStateCallback(const robot_msgs::MechanismStateConstPtr &mechanismState)
{
    bool change = !haveMechanismState_;
    
    static bool first_time = true;
    
    unsigned int n = mechanismState->get_joint_states_size();
    for (unsigned int i = 0 ; i < n ; ++i)
    {
	planning_models::KinematicModel::Joint* joint = kmodel_->getJoint(mechanismState->joint_states[i].name);
	if (joint)
	{
	    if (joint->usedParams == 1)
	    {
		double pos = mechanismState->joint_states[i].position;
		bool this_changed = robotState_->setParamsJoint(&pos, mechanismState->joint_states[i].name);
		change = change || this_changed;
	    }
	    else
	    {
		if (first_time)
		    ROS_WARN("Incorrect number of parameters: %s (expected %d, had 1)", mechanismState->joint_states[i].name.c_str(), joint->usedParams);
	    }
	}
	else
	{
	    if (first_time)
		ROS_ERROR("Unknown joint: %s", mechanismState->joint_states[i].name.c_str());
	}
    }
    
    first_time = false;

    lastStateUpdate_ = ros::Time::now();
    if (!haveMechanismState_)
	haveMechanismState_ = robotState_->seenAll();
    
    if (change && onStateUpdate_ != NULL)
	onStateUpdate_();
}

void planning_environment::KinematicModelStateMonitor::localizedPoseCallback(const robot_msgs::PoseWithCovarianceConstPtr &localizedPose)
{
    tf::PoseMsgToTF(localizedPose->pose, pose_);
    lastPoseUpdate_ = ros::Time::now();

    bool change = !havePose_;
    havePose_ = true;
    
    if (!planarJoints_.empty())
    {
	double planar_joint[3];
	planar_joint[0] = pose_.getOrigin().x();
	planar_joint[1] = pose_.getOrigin().y();
	
	double yaw, pitch, roll;
	pose_.getBasis().getEulerZYX(yaw, pitch, roll);
	planar_joint[2] = yaw;
	
	for (unsigned int i = 0 ; i < planarJoints_.size() ; ++i)
	{
	    bool this_changed = robotState_->setParamsJoint(planar_joint, planarJoints_[i]);
	    change = change || this_changed;
	}
    }
    
    if (!floatingJoints_.empty())
    {
	double floating_joint[7];
	floating_joint[0] = pose_.getOrigin().x();
	floating_joint[1] = pose_.getOrigin().y();
	floating_joint[2] = pose_.getOrigin().z();
	btQuaternion q = pose_.getRotation();	
	floating_joint[3] = q.x();
	floating_joint[4] = q.y();
	floating_joint[5] = q.z();
	floating_joint[6] = q.w();

	for (unsigned int i = 0 ; i < floatingJoints_.size() ; ++i)
	{
	    bool this_changed = robotState_->setParamsJoint(floating_joint, floatingJoints_[i]);
	    change = change || this_changed;
	}
    }
    
    if (change && onStateUpdate_ != NULL)
	onStateUpdate_();
}

void planning_environment::KinematicModelStateMonitor::waitForState(void) const
{
    while (nh_.ok() && !haveMechanismState_)
    {
	ROS_INFO("Waiting for mechanism state ...");	    
	ros::spinOnce();
	ros::Duration().fromSec(0.05).sleep();
    }
    ROS_INFO("Mechanism state received!");
}

void planning_environment::KinematicModelStateMonitor::waitForPose(void) const
{
    while (nh_.ok() && !havePose_)
    {
	ROS_INFO("Waiting for robot pose ...");
	ros::spinOnce();
	ros::Duration().fromSec(0.05).sleep();
    }
    ROS_INFO("Robot pose received!");
}

bool planning_environment::KinematicModelStateMonitor::isStateUpdated(double sec) const
{
    if (sec > 0 && lastStateUpdate_ < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

bool planning_environment::KinematicModelStateMonitor::isPoseUpdated(double sec) const
{
    if (sec > 0 && lastPoseUpdate_ < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

void planning_environment::KinematicModelStateMonitor::printRobotState(void) const
{
    std::stringstream ss;
    robotState_->print(ss);
    ROS_INFO("%s", ss.str().c_str());
}
