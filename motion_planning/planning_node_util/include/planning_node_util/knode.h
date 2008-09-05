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

/**

@mainpage

@htmlinclude ../../manifest.html

@b NodeRobotModel is a class that is also of a given robot model and
uses a ROS node to retrieve the necessary data.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b localizedpose/RobotBase2DOdom : localized position of the robot base

Publishes to (name/type):
- None

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- None

<hr>

@section parameters ROS parameters
- None

**/

#include <ros/node.h>
#include <ros/time.h>
#include <urdf/URDF.h>
#include <planning_models/kinematic.h>
#include <std_msgs/RobotBase2DOdom.h>
#include <rosTF/rosTF.h>
#include <cmath>

#include <mechanism_control/MechanismState.h>

namespace planning_node_util
{
    
    class NodeRobotModel
    {

    public:
	
        NodeRobotModel(ros::node *node, const std::string &robot_model) : m_tf(*node, true, 1 * 1000000000ULL, 1000000000ULL)
	{
	    m_urdf = NULL;
	    m_kmodel = NULL;
	    m_robotState = NULL;
	    m_node = node;
	    m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;
	    m_robotModelName = robot_model;
	    m_haveState = false;
	    m_haveMechanismState = false;
	    m_haveBasePos = false;
	    
	    m_node->subscribe("localizedpose",   m_localizedPose,  &NodeRobotModel::localizedPoseCallback,  this, 1);
	    m_node->subscribe("mechanism_state", m_mechanismState, &NodeRobotModel::mechanismStateCallback, this, 1);
	}

	virtual ~NodeRobotModel(void)
	{
	    if (m_urdf)
		delete m_urdf;
	    if (m_robotState)
		delete m_robotState;
	    if (m_kmodel)
		delete m_kmodel;
	}
	
	void setRobotDescriptionFromData(const char *data)
	{
	    robot_desc::URDF *file = new robot_desc::URDF();
	    if (file->loadString(data))
		setRobotDescription(file);
	    else
		delete file;
	}
	
	void setRobotDescriptionFromFile(const char *filename)
	{
	    robot_desc::URDF *file = new robot_desc::URDF();
	    if (file->loadFile(filename))
		setRobotDescription(file);
	    else
		delete file;
	}
	
	virtual void setRobotDescription(robot_desc::URDF *file)
	{
	    if (m_urdf)
		delete m_urdf;
	    if (m_kmodel)
		delete m_kmodel;
	    
	    m_urdf = file;
	    m_kmodel = new planning_models::KinematicModel();
	    m_kmodel->setVerbose(false);
	    m_kmodel->build(*file);
	    
	    m_robotState = m_kmodel->newStateParams();
	    m_robotState->setAll(0.0);
	}
	
	virtual void loadRobotDescription(void)
	{
	    if (!m_robotModelName.empty() && m_robotModelName != "-")
	    {
		std::string content;
		if (m_node->get_param(m_robotModelName, content))
		    setRobotDescriptionFromData(content.c_str());
		else
		    fprintf(stderr, "Robot model '%s' not found!\n", m_robotModelName.c_str());
	    }
	}
	
	virtual void defaultPosition(void)
	{
	    if (m_kmodel)
		m_kmodel->defaultState();
	}
	
	bool loadedRobot(void) const
	{
	    return m_kmodel != NULL;
	}
	
	void waitForState(void)
	{
	    while (m_node->ok() && (m_haveState ^ loadedRobot()))
		ros::Duration(0.05).sleep();
	}
	
    protected:
	
	void localizedPoseCallback(void)
	{
	    bool success = true;
	    libTF::TFPose2D pose;
	    pose.x = m_localizedPose.pos.x;
	    pose.y = m_localizedPose.pos.y;
	    pose.yaw = m_localizedPose.pos.th;
	    pose.time = m_localizedPose.header.stamp.to_ull();
	    pose.frame = m_localizedPose.header.frame_id;
	    
	    try
	    {
		pose = m_tf.transformPose2D("FRAMEID_MAP", pose);
	    }
	    catch(libTF::TransformReference::LookupException& ex)
	    {
		fprintf(stderr, "Discarding pose: Transform reference lookup exception\n");
		success = false;
	    }
	    catch(libTF::TransformReference::ExtrapolateException& ex)
	    {
		fprintf(stderr, "Discarding pose: Extrapolation exception: %s\n", ex.what());
		success = false;
	    }
	    catch(...)
	    {
		fprintf(stderr, "Discarding pose: Exception in pose computation\n");
		success = false;
	    }
	    
	    if (success)
	    {
		if (isfinite(pose.x))
		    m_basePos[0] = pose.x;
		if (isfinite(pose.y))
		    m_basePos[1] = pose.y;
		if (isfinite(pose.yaw))
		    m_basePos[2] = pose.yaw;
		m_haveBasePos = true;
		baseUpdate();
	    }
	}
	
	void mechanismStateCallback(void)
	{
	    if (m_robotState)
	    {
		unsigned int n = m_mechanismState.get_joint_states_size();
		for (unsigned int i = 0 ; i < n ; ++i)
		{
		    double pos = m_mechanismState.joint_states[i].position;
		    m_robotState->setParams(&pos, m_mechanismState.joint_states[i].name);
		}
	    }
	    m_haveMechanismState = true;
	    stateUpdate();
	}
	
	virtual void baseUpdate(void)
	{
	    if (m_robotState)
		for (unsigned int i = 0 ; i < m_kmodel->getRobotCount() ; ++i)
		{
		    planning_models::KinematicModel::PlanarJoint* pj = 
			dynamic_cast<planning_models::KinematicModel::PlanarJoint*>(m_kmodel->getRobot(i)->chain);
		    if (pj)
			m_robotState->setParams(m_basePos, pj->name);
		}
	    stateUpdate();
	}
	
	virtual void stateUpdate(void)
	{
	    m_haveState = m_haveBasePos && m_haveMechanismState;
	}
	
	rosTFClient                                   m_tf; 
	ros::node                                    *m_node;
	std_msgs::RobotBase2DOdom                     m_localizedPose;
	bool                                          m_haveBasePos;	
	
	mechanism_control::MechanismState             m_mechanismState; // this message should be moved to robot_msgs
	bool                                          m_haveMechanismState;
	
	robot_desc::URDF                             *m_urdf;
	planning_models::KinematicModel              *m_kmodel;
	
	std::string                                   m_robotModelName;
	double                                        m_basePos[3];
	
	planning_models::KinematicModel::StateParams *m_robotState;
	bool                                          m_haveState;
	
    };
     
}
