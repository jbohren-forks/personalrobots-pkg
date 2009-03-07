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

#include "kinematic_planning/ompl_planner/RKPPlannerSetup.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <cassert>
#include <vector>

kinematic_planning::RKPPlannerSetup::RKPPlannerSetup(RKPModelBase *m)
{
    model = m;
    mp = NULL;
    gaik = NULL;
    si = NULL;
    svc = NULL;
    smoother = NULL;
}

kinematic_planning::RKPPlannerSetup::~RKPPlannerSetup(void)
{
    if (mp)
	delete mp;
    if (gaik)
	delete gaik;
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
void kinematic_planning::RKPPlannerSetup::setupDistanceEvaluators(void)
{
    assert(si);
    sde["L2Square"] = new ompl::sb::L2SquareStateDistanceEvaluator(si);
}
	
ompl::base::ProjectionEvaluator* kinematic_planning::RKPPlannerSetup::getProjectionEvaluator(const std::map<std::string, std::string> &options) const
{
    std::map<std::string, std::string>::const_iterator pit = options.find("projection");
    std::map<std::string, std::string>::const_iterator cit = options.find("celldim");
    ompl::base::ProjectionEvaluator *pe = NULL;
    
    if (pit != options.end() && cit != options.end())
    {
	std::string proj = pit->second;
	boost::trim(proj);
	std::string celldim = cit->second;
	boost::trim(celldim);
	
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
	    pe = new ompl::sb::OrthogonalProjectionEvaluator(projection);
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
	ROS_INFO("Projection is set to %s", proj.c_str());
	ROS_INFO("Cell dimensions set to %s", celldim.c_str());
    }
    
    return pe;
}

void kinematic_planning::RKPPlannerSetup::preSetup(const std::map<std::string, std::string> &options)
{
    ROS_INFO("Adding %s instance for motion planning: %s", name.c_str(), model->groupName.c_str());
    
    si       = new SpaceInformationRKPModel(model);
    svc      = new StateValidityPredicate(model);
    si->setStateValidityChecker(svc);
    
    smoother = new ompl::sb::PathSmootherKinematic(si);
    smoother->setMaxSteps(50);
    smoother->setMaxEmptySteps(4);

    gaik     = new ompl::sb::GAIK(si);
}

void kinematic_planning::RKPPlannerSetup::postSetup(const std::map<std::string, std::string> &options)
{
    setupDistanceEvaluators();
    si->setup();
    mp->setup();	    
}

bool kinematic_planning::RKPPlannerSetup::hasOption(const std::map<std::string, std::string> &options, const std::string &opt) const
{
    return options.find(opt) != options.end();
}

double kinematic_planning::RKPPlannerSetup::optionAsDouble(const std::map<std::string, std::string> &options, const std::string &opt, double def) const
{
    std::map<std::string, std::string>::const_iterator it = options.find(opt);
    return (it != options.end()) ? parseDouble(it->second, def) : def;
}

std::string kinematic_planning::RKPPlannerSetup::optionAsString(const std::map<std::string, std::string> &options, const std::string &opt, const::std::string &def) const
{
    std::map<std::string, std::string>::const_iterator it = options.find(opt);
    return (it != options.end()) ? it->second : def;
}

double kinematic_planning::RKPPlannerSetup::parseDouble(const std::string &value, double def) const
{
    try
    {
	return boost::lexical_cast<double>(value);
    }
    catch (...)
    {
	return def;
    }
}
