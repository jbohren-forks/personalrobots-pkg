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
   This class simply allows a node class to be aware of the kinematic
   state the robot is currently in.
*/

#include <ros/node.h>
#include <ros/time.h>
#include <urdf/URDF.h>
#include <planning_models/kinematic.h>
#include <tf/transform_listener.h>
#include <cmath>

#include <robot_msgs/MechanismState.h>

/** Main namespace */
namespace kinematic_planning
{
    /**
       @b KinematicStateMonitor is a class that is also of a given robot model and
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
       
    **/
    class KinematicStateMonitor
    {
	
    public:
        planning_models::KinematicModel* getKModel(void) { return m_kmodel; }
        planning_models::KinematicModel::StateParams* getRobotState() { return m_robotState; }
	
        KinematicStateMonitor(ros::Node *node, const std::string &robot_model_name) : m_tf(*node, true, 1000000000ULL)
	{
	    m_tf.setExtrapolationLimit(ros::Duration().fromSec(10));
	    m_robotModelName = robot_model_name;
	    m_urdf = NULL;
	    m_kmodel = NULL;
	    m_robotState = NULL;
	    m_node = node;
	    m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;

	    m_haveState = false;
	    m_haveMechanismState = false;
	    m_haveBasePos = false;
	    
	    //	    m_node->subscribe("localizedpose",   m_localizedPose,  &KinematicStateMonitor::localizedPoseCallback,  this, 1);
	    m_node->subscribe("mechanism_state", m_mechanismState, &KinematicStateMonitor::mechanismStateCallback, this, 1);
	}
	
	virtual ~KinematicStateMonitor(void)
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
	    
	    m_haveState = false;
	    m_haveMechanismState = false;
	    m_haveBasePos = false;
	}
	
	virtual void loadRobotDescription(void)
	{
	    if (!m_robotModelName.empty() && m_robotModelName != "-")
	    {
		std::string content;
		if (m_node->getParam(m_robotModelName, content))
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
		ros::Duration().fromSec(0.05).sleep();
	}
	
	virtual void stateUpdate(void)
	{
	    m_haveState = m_haveBasePos && m_haveMechanismState;
	}
    protected:
	/*
	void localizedPoseCallback(void)
	{
	    bool success = true;
	    tf::Stamped<tf::Pose> pose (tf::Transform(tf::Quaternion(m_localizedPose.pos.th, 0, 0),
                                                      tf::Point(m_localizedPose.pos.x,
                                                                m_localizedPose.pos.y, 0)),
                                        m_localizedPose.header.stamp,
                                        m_localizedPose.header.frame_id);
	    try
	    {
		m_tf.transformPose("map", pose, pose);
	    }
	    catch(tf::LookupException& ex)
	    {
	        fprintf(stderr, "Discarding pose: from %s, Transform reference lookup exception: %s\n", pose.frame_id_.c_str(), ex.what());
		success = false;
	    }
	    catch(tf::ExtrapolationException& ex)
	    {
		fprintf(stderr, "Discarding pose: from %s, Extrapolation exception: %s\n", pose.frame_id_.c_str(), ex.what());
		success = false;
	    }
	    catch(tf::ConnectivityException& ex)
	    {
		fprintf(stderr, "Discarding pose: from %s, Connectivity exception: %s\n", pose.frame_id_.c_str(), ex.what());
		success = false;
	    }
	    catch(tf::TransformException)
	    {
		fprintf(stderr, "Discarding pose: from %s, Exception in pose computation\n", pose.frame_id_.c_str());
		success = false;
	    }
	    
	    if (success)
	    {
		if (std::isfinite(pose.getOrigin().x()))
		    m_basePos[0] = pose.getOrigin().x();
		if (std::isfinite(pose.getOrigin().y()))
		    m_basePos[1] = pose.getOrigin().y();
                double yaw, pitch, roll;
                pose.getBasis().getEulerZYX(yaw, pitch, roll);
		if (std::isfinite(yaw))
		    m_basePos[2] = yaw;
		m_haveBasePos = true;
		baseUpdate();
	    }
	}
	*/
	void mechanismStateCallback(void)
	{
	    bool change = false;
	    m_haveMechanismState = true;
	    if (m_robotState)
	    {
		unsigned int n = m_mechanismState.get_joint_states_size();
		for (unsigned int i = 0 ; i < n ; ++i)
		{
		    double pos = m_mechanismState.joint_states[i].position;
		    change = change || m_robotState->setParams(&pos, m_mechanismState.joint_states[i].name);
		}
	    }
	    if (change)
		stateUpdate();
	}
	
	virtual void baseUpdate(void)
	{
	    bool change = false;
	    if (m_robotState)
		for (unsigned int i = 0 ; i < m_kmodel->getRobotCount() ; ++i)
		{
		    planning_models::KinematicModel::PlanarJoint* pj = 
			dynamic_cast<planning_models::KinematicModel::PlanarJoint*>(m_kmodel->getRobot(i)->chain);
		    if (pj)
		        change = change || m_robotState->setParams(m_basePos, pj->name);
		}
	    if (change)
		stateUpdate();
	}
	
	ros::Node                                    *m_node;
	tf::TransformListener                         m_tf; 
	robot_desc::URDF                             *m_urdf;
	std::string                                   m_robotModelName;
	planning_models::KinematicModel              *m_kmodel;


	//	std_msgs::RobotBase2DOdom                     m_localizedPose;
	// some pose message for the base; the transform from map to base
	bool                                          m_haveBasePos;	
	double                                        m_basePos[3];
	
	robot_msgs::MechanismState                    m_mechanismState;
	bool                                          m_haveMechanismState;
	
	/** The complete robot state (MAP frame) */
	planning_models::KinematicModel::StateParams *m_robotState;
	bool                                          m_haveState;
	
    };
    
}
