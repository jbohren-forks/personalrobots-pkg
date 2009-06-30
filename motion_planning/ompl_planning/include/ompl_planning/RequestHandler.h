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

#ifndef OMPL_PLANNING_REQUEST_HANDLER_
#define OMPL_PLANNING_REQUEST_HANDLER_

#include "ompl_planning/Model.h"
#include <motion_planning_srvs/KinematicPlan.h>

/** \brief Main namespace */
namespace ompl_planning
{    
   
    /** \brief This class represents a basic request to a motion
	planner. */

    class RequestHandler
    {
    public:
	
	RequestHandler(void)
	{
	}
	
	~RequestHandler(void)
	{
	}
	
	bool isRequestValid(ModelMap &models, motion_planning_srvs::KinematicPlan::Request &req);

	/* \brief Check and compute a motion plan. Return true if the plan was succesfully computed */
	bool computePlan(ModelMap &models, const planning_models::StateParams *start, motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res);
	
	
    protected:

	/** \brief Set up all the data needed by motion planning based on a request and lock the planner setup
	 *  using this data */
	void configure(const planning_models::StateParams *startState, motion_planning_srvs::KinematicPlan::Request &req, PlannerSetup *psetup);
	
	/** \brief Compute the actual motion plan. Return true if computed plan was trivial (start state already in goal region) */
	bool callPlanner(PlannerSetup *psetup, int times, double allowed_time, bool interpolate,
			 ompl::base::Path* &bestPath, double &bestDifference, bool &approximate);
	
    };
    
} // ompl_planning

#endif
