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
#include <tf/transform_listener.h>
#include <mechanism_msgs/MechanismState.h>
#include <boost/bind.hpp>
#include <vector>
#include <string>

namespace planning_environment
{

    /** \brief @b KinematicModelStateMonitor is a class that monitors the robot state for the kinematic model defined in @b RobotModels
	If the pose is not included, the robot state is the frame of the link it attaches to the world. If the pose is included,
	the frame of the robot is the one in which the pose is published.
    */
    class KinematicModelStateMonitor
    {
    public:

	KinematicModelStateMonitor(RobotModels *rm)
	{
	    rm_ = rm;
	    includePose_ = false;
	    setupRSM();
	}

	KinematicModelStateMonitor(RobotModels *rm, const std::string &frame_id)
	{
	    rm_ = rm;
	    includePose_ = true;
	    frame_id_ = frame_id;
	    setupRSM();
	}

	virtual ~KinematicModelStateMonitor(void)
	{
	    if (robotState_)
		delete robotState_;
	    if (tf_)
		delete tf_;
	}

	/** \brief Define a callback for when the state is changed */
	void setOnStateUpdateCallback(const boost::function<void(void)> &callback)
	{
	    onStateUpdate_ = callback;
	}

	/** \brief Get the kinematic model that is being monitored */
	planning_models::KinematicModel* getKinematicModel(void) const
	{
	    return kmodel_;
	}

	/** \brief Get the instance of @b RobotModels that is being used */
	RobotModels* getRobotModels(void) const
	{
	    return rm_;
	}

	/** \brief Return a pointer to the maintained robot state */
	const planning_models::StateParams* getRobotState(void) const
	{
	    return robotState_;
	}

	/** \brief Return the frame id of the state */
	const std::string& getFrameId(void) const
	{
	    return frame_id_;
	}

	/** \brief Return the robot frame id (robot without pose) */
	const std::string& getRobotFrameId(void) const
	{
	    return robot_frame_;
	}

	/** \brief Return true if a full mechanism state has been received */
	bool haveState(void) const
	{
	    return haveMechanismState_ && (!includePose_ || (includePose_ && havePose_));
	}

	/** \brief Return the time of the last state update */
	const ros::Time& lastStateUpdate(void) const
	{
	    return lastStateUpdate_;
	}

	/** \brief Wait until a full mechanism state is received */
	void waitForState(void) const;

	/** \brief Return true if a full mechanism state has been received in the last sec seconds. If sec < 10us, this function always returns true. */
	bool isStateUpdated(double sec) const;

	/** \brief Return true if the pose is included in the state */
	bool isPoseIncluded(void) const
	{
	    return includePose_;
	}

	/** \brief Output the current state as ROS INFO */
	void printRobotState(void) const;

    protected:

	void setupRSM(void);
	void mechanismStateCallback(const mechanism_msgs::MechanismStateConstPtr &mechanismState);


	RobotModels                     *rm_;
	bool                             includePose_;
	planning_models::KinematicModel *kmodel_;
	std::string                      planarJoint_;
	std::string                      floatingJoint_;

	ros::NodeHandle                  nh_;
	ros::Subscriber                  mechanismStateSubscriber_;
	tf::TransformListener           *tf_;

	planning_models::StateParams    *robotState_;
	tf::Pose                         pose_;
	std::string                      robot_frame_;
	std::string                      frame_id_;

	boost::function<void(void)>      onStateUpdate_;

	bool                             havePose_;
	bool                             haveMechanismState_;
	ros::Time                        lastStateUpdate_;
    };

}

#endif
