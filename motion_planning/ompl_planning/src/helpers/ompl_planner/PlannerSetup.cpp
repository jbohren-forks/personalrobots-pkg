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

#include "ompl_planning/planners/PlannerSetup.h"
#include <boost/algorithm/string.hpp>
#include <vector>
#include <cassert>

ompl_planning::PlannerSetup::PlannerSetup(ModelBase *m)
{
    model = m;
    mp = NULL;
    si = NULL;
    svc = NULL;
    smoother = NULL;
    priority = 0;
}

ompl_planning::PlannerSetup::~PlannerSetup(void)
{
    if (mp)
	delete mp;
    if (svc)
	delete svc;
    for (std::map<std::string, ompl::base::StateDistanceEvaluator*>::iterator j = sde.begin(); j != sde.end() ; ++j)
	if (j->second)
	    delete j->second;
    if (smoother)
	delete smoother;
    if (si)
	delete si;
}

/** For each planner definition, define the set of distance metrics it can use */
void ompl_planning::PlannerSetup::setupDistanceEvaluators(void)
{
    assert(si);
    sde["L2Square"] = new ompl::base::L2SquareStateDistanceEvaluator(si);
}
	
ompl::base::ProjectionEvaluator* ompl_planning::PlannerSetup::getProjectionEvaluator(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options) const
{
    ompl::base::ProjectionEvaluator *pe = NULL;
    
    if (options->hasParam("projection") && options->hasParam("celldim"))
    {
	std::string proj = options->getParamString("projection");
	std::string celldim = options->getParamString("celldim");
	
	if (proj.substr(0, 4) == "link")
	{
	    std::string linkName = proj.substr(4);
	    boost::trim(linkName);
	    pe = new LinkPositionProjectionEvaluator(model, linkName);
	}
	else
	{
	    std::vector<unsigned int> projection;
	    std::stringstream ss(proj);
	    while (ss.good())
	    {
		unsigned int comp;
		ss >> comp;
		projection.push_back(comp);
	    }
	    pe = new ompl::base::OrthogonalProjectionEvaluator(projection);
	}
	
	std::vector<double> cdim;
	std::stringstream ss(celldim);
	while (ss.good())
	{
	    double comp;
	    ss >> comp;
	    cdim.push_back(comp);
	}
	
	pe->setCellDimensions(cdim);
	ROS_DEBUG("Projection is set to %s", proj.c_str());
	ROS_DEBUG("Cell dimensions set to %s", celldim.c_str());
    }
    
    return pe;
}

void ompl_planning::PlannerSetup::preSetup(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    ROS_DEBUG("Adding %s instance for motion planning: %s", name.c_str(), model->groupName.c_str());
    bool dynamic = options->getParamString("type")[0] == 'd';
    
    if (dynamic)
    {
	SpaceInformationDynamicModel *sd = new SpaceInformationDynamicModel(model);
	svc = new StateValidityPredicate(sd, model);
	si = sd;
    }
    else
    {
	SpaceInformationKinematicModel *sk = new SpaceInformationKinematicModel(model);
	svc = new StateValidityPredicate(sk, model);
	si = sk;
    }
    
    si->setStateValidityChecker(svc);

    if (!dynamic)
    {
	smoother = new ompl::kinematic::PathSmootherKinematic(dynamic_cast<ompl::kinematic::SpaceInformationKinematic*>(si));
	smoother->setMaxSteps(50);
	smoother->setMaxEmptySteps(4);
    }
}

void ompl_planning::PlannerSetup::postSetup(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options)
{
    setupDistanceEvaluators();
    si->setup();
    mp->setup();	    
}
