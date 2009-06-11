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

#ifndef KINEMATIC_PLANNING_RKP_REQUEST_HANDLER_
#define KINEMATIC_PLANNING_RKP_REQUEST_HANDLER_

#include "kinematic_planning/RKPModel.h"
#include <motion_planning_msgs/KinematicPath.h>
#include <motion_planning_srvs/KinematicPlan.h>

#include <ros/console.h>
#include <iostream>
#include <sstream>

/** Main namespace */
namespace kinematic_planning
{    
   
    /** This class represents a basic request to a kinematic motion
	planner. */

    class RKPRequestHandler
    {
    public:
	
	RKPRequestHandler(void)
	{
	    m_activePSetup = NULL;
	}
	
	~RKPRequestHandler(void)
	{
	}
	
	/** Set up all the data needed by motion planning based on a request and lock the planner setup
	 *  using this data */
	bool configure(ModelMap &models, const motion_planning_msgs::KinematicState &startState, const motion_planning_srvs::KinematicPlan::Request &req);
	void release(void);
	
	bool isActive(void);
	RKPPlannerSetup *activePlannerSetup(void);
	motion_planning_srvs::KinematicPlan::Request& activeRequest(void);
	motion_planning_msgs::KinematicState& activeStartState(void);
	
	bool isStillValid(motion_planning_msgs::KinematicPath &path);
	bool isTrivial(double *distance = NULL);
	void execute(motion_planning_msgs::KinematicPath &path, double &distance, bool &trivial, bool &approximate);

    protected:

	bool isRequestValid(ModelMap &models, motion_planning_msgs::KinematicState &startState, motion_planning_srvs::KinematicPlan::Request &req);
	bool areSpaceParamsValid(const ModelMap &modelsRef, motion_planning_msgs::KinematicSpaceParameters &params) const;

	
	void update(void);
	void setupGoalState(RKPPlannerSetup *psetup, motion_planning_srvs::KinematicPlan::Request &req);

	/** Configure the state space for a given set of parameters and set the starting state */
	void setupStateSpaceAndStartState(RKPPlannerSetup *psetup,
					  motion_planning_msgs::KinematicSpaceParameters &params,
					  motion_planning_msgs::KinematicState &start_state);
	
	/** Set the kinematic constraints to follow */
	void setupConstraints(RKPPlannerSetup *psetup, const motion_planning_msgs::KinematicConstraints &cstrs);
	
	/** Compute the actual motion plan. Return true if computed plan was trivial (start state already in goal region) */
	bool computePlan(RKPPlannerSetup *psetup, int times, double allowed_time, bool interpolate,
			 ompl::sb::PathKinematic* &bestPath, double &bestDifference, bool &approximate);
	void fillSolution(RKPPlannerSetup *psetup, ompl::sb::PathKinematic *bestPath, double bestDifference,
			  motion_planning_msgs::KinematicPath &path, double &distance);
	void cleanupPlanningData(RKPPlannerSetup *psetup);
	
	motion_planning_srvs::KinematicPlan::Request m_activeReq;
	motion_planning_msgs::KinematicState         m_activeStartState;
	RKPPlannerSetup                             *m_activePSetup;
    };
    
} // kinematic_planning

#endif
