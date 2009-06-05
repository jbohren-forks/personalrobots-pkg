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

#include "kinematic_planning/KinematicStateMonitor.h"
#include <sstream>

void kinematic_planning::KinematicStateMonitor::kinematicStateSubscribe(void)
{
    m_mechanismStateSubscriber = m_nodeHandle.subscribe("mechanism_state", 1, &KinematicStateMonitor::mechanismStateCallback, this);
    m_localizedPoseSubscriber  = m_nodeHandle.subscribe("localized_pose",  1, &KinematicStateMonitor::localizedPoseCallback,  this);
}

boost::shared_ptr<planning_models::KinematicModel> kinematic_planning::KinematicStateMonitor::getKModel(void) const
{ 
    return m_kmodel;
}

boost::shared_ptr<planning_models::KinematicModel::StateParams> kinematic_planning::KinematicStateMonitor::getRobotState(void) const
{
    return m_robotState;
}

void kinematic_planning::KinematicStateMonitor::setIncludeBaseInState(bool value)
{
    m_includeBaseInState = value;
}

void kinematic_planning::KinematicStateMonitor::loadRobotDescription(void)
{
    if (m_nodeHandle.hasParam("robot_description"))
    {
	m_envModels = boost::shared_ptr<planning_environment::CollisionModels>(new planning_environment::CollisionModels("robot_description"));
	m_urdf = m_envModels->getParsedDescription();
	m_kmodel = m_envModels->getKinematicModel();
	m_robotState = boost::shared_ptr<planning_models::KinematicModel::StateParams>(m_kmodel->newStateParams());
    }
    else
	ROS_ERROR("Robot model not found! Did you remap robot_description?");
}

bool kinematic_planning::KinematicStateMonitor::loadedRobot(void) const
{
    return m_kmodel;
}

void kinematic_planning::KinematicStateMonitor::waitForState(void)
{
    while (m_nodeHandle.ok() && (m_haveMechanismState ^ loadedRobot()))
    {
	ROS_INFO("Waiting for mechanism state ...");	    
	ros::Duration().fromSec(0.05).sleep();
    }
    ROS_INFO("Mechanism state received!");
}

void kinematic_planning::KinematicStateMonitor::waitForPose(void)
{
    ROS_INFO("Waiting for robot pose ...");	    
    while (m_nodeHandle.ok() && (m_haveBasePos ^ loadedRobot()))
	ros::Duration().fromSec(0.05).sleep();
    ROS_INFO("Robot pose received!");
}

void kinematic_planning::KinematicStateMonitor::printCurrentState(void)
{
    std::stringstream ss;
    m_robotState->print(ss);
    ROS_INFO("%s", ss.str().c_str());
}

bool kinematic_planning::KinematicStateMonitor::isStateUpdated(double sec)
{
    if (sec > 0 && m_lastStateUpdate < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

bool kinematic_planning::KinematicStateMonitor::isBaseUpdated(double sec)
{
    if (sec > 0 && m_lastBaseUpdate < ros::Time::now() - ros::Duration(sec))
	return false;
    else
	return true;
}

void kinematic_planning::KinematicStateMonitor::stateUpdate(void)
{
}

void kinematic_planning::KinematicStateMonitor::baseUpdate(void)
{
    bool change = false;
    if (m_robotState && m_includeBaseInState)
	for (unsigned int i = 0 ; i < m_kmodel->getRobotCount() ; ++i)
	{
	    planning_models::KinematicModel::PlanarJoint* pj = 
		dynamic_cast<planning_models::KinematicModel::PlanarJoint*>(m_kmodel->getRobot(i)->chain);
	    if (pj)
	    {
		bool this_changed = m_robotState->setParamsJoint(m_basePos, pj->name);
		change = change || this_changed;
	    }
	}
    if (change)
	stateUpdate();
}

void kinematic_planning::KinematicStateMonitor::localizedPoseCallback(const robot_msgs::PoseWithCovarianceConstPtr &localizedPose)
{
    tf::PoseMsgToTF(localizedPose->pose, m_pose);
    if (std::isfinite(m_pose.getOrigin().x()))
	m_basePos[0] = m_pose.getOrigin().x();
    if (std::isfinite(m_pose.getOrigin().y()))
	m_basePos[1] = m_pose.getOrigin().y();
    double yaw, pitch, roll;
    m_pose.getBasis().getEulerZYX(yaw, pitch, roll);
    if (std::isfinite(yaw))
	m_basePos[2] = yaw;
    m_haveBasePos = true;
    m_lastBaseUpdate = ros::Time::now();
    baseUpdate();
}

void kinematic_planning::KinematicStateMonitor::mechanismStateCallback(const robot_msgs::MechanismStateConstPtr &mechanismState)
{
    bool change = false;

    if (m_robotState)
    {
	static bool first_time = true;

	unsigned int n = mechanismState->get_joint_states_size();
	for (unsigned int i = 0 ; i < n ; ++i)
	{
	    planning_models::KinematicModel::Joint* joint = m_kmodel->getJoint(mechanismState->joint_states[i].name);
	    if (joint)
	    {
		if (joint->usedParams == 1)
		{
		    double pos = mechanismState->joint_states[i].position;
		    bool this_changed = m_robotState->setParamsJoint(&pos, mechanismState->joint_states[i].name);
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
	
	if (!m_haveMechanismState)
	    m_haveMechanismState = m_robotState->seenAll();
	m_lastStateUpdate = ros::Time::now();
    }
    if (change)
	stateUpdate();
}
