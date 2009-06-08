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

#ifndef PLANNING_ENVIRONMENT_KINEMATIC_MODEL_STATE_MONITOR_
#define PLANNING_ENVIRONMENT_KINEMATIC_MODEL_STATE_MONITOR_

#include "planning_environment/robot_models.h"
#include <tf/transform_datatypes.h>
#include <robot_msgs/MechanismState.h>
#include <robot_msgs/PoseWithCovariance.h>
#include <boost/bind.hpp>
#include <vector>
#include <string>

namespace planning_environment
{

    /** @b KinematicModelStateMonitor is a class that monitors the robot state for the kinematic model defined in @b RobotModels

       <hr>
       
       @section topic ROS topics
       
       Subscribes to (name/type):
       - @b "mechanism_model"/MechanismModel : position for each of the robot's joints
       - @b "localized_pose"/PoseWithCovariance : localized robot pose
       
    */    
    class KinematicModelStateMonitor
    {
    public:
	
	KinematicModelStateMonitor(RobotModels *rm, bool includePose = false)
	{
	    rm_ = rm;
	    includePose_ = includePose;
	    setupRSM();
	}
	
	virtual ~KinematicModelStateMonitor(void)
	{
	    if (robotState_)
		delete robotState_;
	}

	/** Define a callback for when the state is changed */
	void setOnStateUpdateCallback(const boost::function<void(void)> &callback)
	{
	    onStateUpdate_ = callback;
	}
	
	planning_models::KinematicModel* getKinematicModel(void) const
	{
	    return kmodel_;
	}
	
	RobotModels* getRobotModels(void) const
	{
	    return rm_;
	}
	
	/** Return a pointer to the maintained robot state */
	const planning_models::KinematicModel::StateParams* getRobotState(void) const
	{
	    return robotState_;
	}
	
	/** Return true if a pose has been received */
	bool havePose(void) const	    
	{
	    return havePose_;
	}

	/** Return true if a full mechanism state has been received */
	bool haveState(void) const	    
	{
	    return haveMechanismState_;
	}
	
	/** Wait until a pose is received */
	void waitForPose(void) const;

	/** Wait until a full mechanism state is received */
	void waitForState(void) const;

	/** Return true if a pose has been received in the last sec seconds */
	bool isPoseUpdated(double sec) const;

	/** Return true if a full mechanism state has been received in the last sec seconds */
	bool isStateUpdated(double sec) const;
	
	/** Output the current state as ROS INFO */
	void printRobotState(void) const;

    protected:

	void setupRSM(void);
	void localizedPoseCallback(const robot_msgs::PoseWithCovarianceConstPtr &localizedPose);
	void mechanismStateCallback(const robot_msgs::MechanismStateConstPtr &mechanismState);


	RobotModels                                  *rm_;
	bool                                          includePose_;
	planning_models::KinematicModel              *kmodel_;
	std::vector<std::string> planarJoints_;
	std::vector<std::string> floatingJoints_;
	
	ros::NodeHandle                               nh_;
	ros::Subscriber                               mechanismStateSubscriber_;	
	ros::Subscriber                               localizedPoseSubscriber_;

	planning_models::KinematicModel::StateParams *robotState_;
	tf::Pose                                      pose_;
	boost::function<void(void)>                   onStateUpdate_;
	
	bool                                          havePose_;
	bool                                          haveMechanismState_;
	ros::Time                                     lastStateUpdate_;
	ros::Time                                     lastPoseUpdate_;
    };
    
	
}

#endif

