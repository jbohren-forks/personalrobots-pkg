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

#include "ompl_planning/Model.h"
#include "ompl_planning/planners/kinematicRRTSetup.h"
#include "ompl_planning/planners/kinematicpRRTSetup.h"
#include "ompl_planning/planners/kinematicLazyRRTSetup.h"
#include "ompl_planning/planners/kinematicSBLSetup.h"
#include "ompl_planning/planners/kinematicESTSetup.h"
#include "ompl_planning/planners/kinematicIKSBLSetup.h"
#include "ompl_planning/planners/kinematicKPIECESetup.h"
#include "ompl_planning/planners/kinematicLBKPIECESetup.h"

#include "ompl_planning/planners/dynamicRRTSetup.h"
#include "ompl_planning/planners/dynamicKPIECESetup.h"

/* instantiate the planners that can be used  */
void ompl_planning::Model::createMotionPlanningInstances(std::vector< boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> > cfgs)
{	
    
    for (unsigned int i = 0 ; i < cfgs.size() ; ++i)
    {
	std::string type = cfgs[i]->getParamString("type");
	
	if (type == "kinematic::RRT")
	    add_planner<kinematicRRTSetup>(cfgs[i]);
	else
	if (type == "kinematic::pRRT")
	    add_planner<kinematicpRRTSetup>(cfgs[i]);
	else
	if (type == "kinematic::LazyRRT")
	    add_planner<kinematicLazyRRTSetup>(cfgs[i]);
	else
	if (type == "kinematic::EST")
	    add_planner<kinematicESTSetup>(cfgs[i]);
	else
	if (type == "kinematic::SBL")
	    add_planner<kinematicSBLSetup>(cfgs[i]);
	else
	if (type == "kinematic::KPIECE")
	    add_planner<kinematicKPIECESetup>(cfgs[i]);
	else
	if (type == "kinematic::LBKPIECE")
	    add_planner<kinematicLBKPIECESetup>(cfgs[i]);
	else
	if (type == "kinematic::IKSBL")
	    add_planner<kinematicIKSBLSetup>(cfgs[i]);
	else
	if (type == "dynamic::RRT")
	    add_planner<dynamicRRTSetup>(cfgs[i]);
	else
	if (type == "dynamic::KPIECE")
	    add_planner<dynamicKPIECESetup>(cfgs[i]);
	else
	    ROS_WARN("Unknown planner type: %s", type.c_str());
    }
}

template<typename _T>
void ompl_planning::Model::add_planner(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    PlannerSetup *p = new _T(dynamic_cast<ModelBase*>(this));
    if (p->setup(options))
	planners[p->name + "[" + options->getName() + "]"] = p;
    else
	delete p;
}

