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
#include <motion_planning_srvs/KinematicPlan.h>

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
	}
	
	~RKPRequestHandler(void)
	{
	}
	
	bool isRequestValid(ModelMap &models, motion_planning_srvs::KinematicPlan::Request &req);

	/* Check and compute a motion plan. Return true if the plan was succesfully computed */
	bool computePlan(ModelMap &models, const planning_models::StateParams *start, motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res);
	
	
    protected:

	/** Set up all the data needed by motion planning based on a request and lock the planner setup
	 *  using this data */
	void configure(const planning_models::StateParams *startState, motion_planning_srvs::KinematicPlan::Request &req, RKPPlannerSetup *psetup);
	
	/** Compute the actual motion plan. Return true if computed plan was trivial (start state already in goal region) */
	bool callPlanner(RKPPlannerSetup *psetup, int times, double allowed_time, bool interpolate,
			 ompl::sb::PathKinematic* &bestPath, double &bestDifference, bool &approximate);
	
    };
    
} // kinematic_planning

#endif
