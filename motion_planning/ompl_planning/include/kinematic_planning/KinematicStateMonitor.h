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

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

#include <urdf/URDF.h>
#include <planning_environment/collision_models.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <cmath>

#include <robot_msgs/MechanismState.h>
#include <robot_msgs/PoseWithCovariance.h>

/** Main namespace */
namespace kinematic_planning
{
    /**
       @b KinematicStateMonitor is a class that is also of a given robot model and
       uses a ROS node to retrieve the necessary data.
       
       <hr>
       
       @section topic ROS topics
       
       Subscribes to (name/type):
       - @b "mechanism_model"/MechanismModel : position for each of the robot's joints
       - @b "localized_pose"/PoseWithCovariance : localized robot pose

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
       - @b "robot_description"/string : the URDF description of the robot we are monitoring

    **/
    
    class KinematicStateMonitor
    {
	
    public:
		
        KinematicStateMonitor(void) : m_tf(*ros::Node::instance(), true, ros::Duration(10))
	{
	    m_tf.setExtrapolationLimit(ros::Duration().fromSec(10));
	    
	    m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;	    
	    m_includeBaseInState = false;	    
	    m_haveMechanismState = false;
	    m_haveBasePos = false;
	    
	    kinematicStateSubscribe();
	}
	
	virtual ~KinematicStateMonitor(void)
	{
	}
	
	boost::shared_ptr<planning_models::KinematicModel>              getKModel(void) const;
	boost::shared_ptr<planning_models::KinematicModel::StateParams> getRobotState(void) const;
	
	void setIncludeBaseInState(bool value);
	
	virtual void loadRobotDescription(void);
	
	bool loadedRobot(void) const;
	void waitForState(void);
	void waitForPose(void);

	void printCurrentState(void);
	bool isStateUpdated(double sec);
	bool isBaseUpdated(double sec);

    protected:
	
	void kinematicStateSubscribe(void);
	
	virtual void stateUpdate(void);
	virtual void baseUpdate(void);
	
	void localizedPoseCallback(const robot_msgs::PoseWithCovarianceConstPtr &localizedPose);
	void mechanismStateCallback(const robot_msgs::MechanismStateConstPtr &mechanismState);
	
	ros::NodeHandle                                    m_nodeHandle;
	ros::Subscriber                                    m_mechanismStateSubscriber;
	ros::Subscriber                                    m_localizedPoseSubscriber;

	tf::TransformListener                              m_tf; 
	
	boost::shared_ptr<planning_environment::CollisionModels>
	                                                   m_envModels;

	boost::shared_ptr<robot_desc::URDF>                m_urdf;
	boost::shared_ptr<planning_models::KinematicModel> m_kmodel;
	
	// info about the pose; this is not placed in the robot's kinematic state 
	bool                                               m_haveBasePos;	
	double                                             m_basePos[3];
	tf::Pose                                           m_pose;
	
	// info about the robot's joints
	bool                                               m_haveMechanismState;	
	boost::shared_ptr<planning_models::KinematicModel::StateParams>
	                                                   m_robotState;
	
	// if this flag is true, the base position is included in the state as well
	bool                                               m_includeBaseInState;
	
	ros::Time                                          m_lastStateUpdate;
	ros::Time                                          m_lastBaseUpdate;
    };
    
} // kinematic_planning
