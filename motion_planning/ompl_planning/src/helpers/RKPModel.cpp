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

#include "kinematic_planning/RKPModel.h"

	/* instantiate the planners that can be used  */
void kinematic_planning::RKPModel::createMotionPlanningInstances(std::vector< boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> > cfgs)
{	
    
    for (unsigned int i = 0 ; i < cfgs.size() ; ++i)
    {
	std::string type = cfgs[i]->getParamString("type");
	if (type == "RRT")
	    addRRT(cfgs[i]);
	else
	    if (type == "LazyRRT")
		addLazyRRT(cfgs[i]);
	    else
		if (type == "EST")
		    addEST(cfgs[i]);
		else
		    if (type == "SBL")
			addSBL(cfgs[i]);
		    else
			if (type == "KPIECE")
			    addKPIECE(cfgs[i]);
			else
			    if (type == "IKSBL")
				addIKSBL(cfgs[i]);
			    else
				if (type == "IKKPIECE")
				    addIKKPIECE(cfgs[i]);
				else
				    ROS_WARN("Unknown planner type: %s", type.c_str());
    }
}

void kinematic_planning::RKPModel::addRRT(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *rrt = new RKPRRTSetup(dynamic_cast<RKPModelBase*>(this));
    if (rrt->setup(options))
	planners[rrt->name] = rrt;
    else
	delete rrt;
}

void kinematic_planning::RKPModel::addLazyRRT(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *rrt = new RKPLazyRRTSetup(dynamic_cast<RKPModelBase*>(this));
    if (rrt->setup(options))
	planners[rrt->name] = rrt;
    else
	delete rrt;
}

void kinematic_planning::RKPModel::addEST(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *est = new RKPESTSetup(dynamic_cast<RKPModelBase*>(this));
    if (est->setup(options))
	planners[est->name] = est;
    else
	delete est;
}

void kinematic_planning::RKPModel::addSBL(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *sbl = new RKPSBLSetup(dynamic_cast<RKPModelBase*>(this));
    if (sbl->setup(options))
	planners[sbl->name] = sbl;
    else
	delete sbl;
}

void kinematic_planning::RKPModel::addIKSBL(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *sbl = new RKPIKSBLSetup(dynamic_cast<RKPModelBase*>(this));
    if (sbl->setup(options))
	planners[sbl->name] = sbl;
    else
	delete sbl;
}

void kinematic_planning::RKPModel::addKPIECE(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *kpiece = new RKPKPIECESetup(dynamic_cast<RKPModelBase*>(this));
    if (kpiece->setup(options))
	planners[kpiece->name] = kpiece;
    else
	delete kpiece;
}


void kinematic_planning::RKPModel::addIKKPIECE(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    RKPPlannerSetup *kpiece = new RKPIKKPIECESetup(dynamic_cast<RKPModelBase*>(this));
    if (kpiece->setup(options))
	planners[kpiece->name] = kpiece;
    else
	delete kpiece;
}
