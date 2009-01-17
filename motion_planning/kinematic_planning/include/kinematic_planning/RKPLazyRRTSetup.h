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

#ifndef KINEMATIC_PLANNING_RKP_LAZY_RRT_SETUP_
#define KINEMATIC_PLANNING_RKP_LAZY_RRT_SETUP_

#include "kinematic_planning/RKPPlannerSetup.h"
#include <ompl/extension/samplingbased/kinematic/extension/rrt/LazyRRT.h>

namespace kinematic_planning
{
    
    class RKPLazyRRTSetup : public RKPPlannerSetup
    {
    public:
	
    RKPLazyRRTSetup(void) : RKPPlannerSetup()
	{
	}
	
	virtual ~RKPLazyRRTSetup(void)
	{
	}
	
	virtual bool setup(RKPModelBase *model, std::map<std::string, std::string> &options)
	{
	    std::cout << "Adding Lazy RRT instance for motion planning: " << model->groupName << std::endl;
	    
	    si       = new SpaceInformationRKPModel(model);
	    svc      = new StateValidityPredicate(model);
	    si->setStateValidityChecker(svc);
	    
	    smoother = new ompl::PathSmootherKinematic(si);
	    smoother->setMaxSteps(50);
	    smoother->setMaxEmptySteps(4);
	    
	    ompl::LazyRRT_t rrt = new ompl::LazyRRT(si);
	    mp                  = rrt;
	    
	    if (options.find("range") != options.end())
	    {
		double range = string_utils::fromString<double>(options["range"]);
		rrt->setRange(range);
		std::cout << "Range is set to " << range << std::endl;		
	    }
	    
	    if (options.find("goal_bias") != options.end())
	    {	
		double bias = string_utils::fromString<double>(options["goal_bias"]);
		rrt->setGoalBias(bias);
		std::cout << "Goal bias is set to " << bias << std::endl;
	    }
	    
	    setupDistanceEvaluators();
	    si->setup();
	    mp->setup();
	    
	    return true;
	}
	
    };
}

#endif
    
