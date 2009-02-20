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

#ifndef KINEMATIC_PLANNING_KINEMATIC_RKP_MODEL_
#define KINEMATIC_PLANNING_KINEMATIC_RKP_MODEL_

#include "kinematic_planning/RKPModelBase.h"

#include "kinematic_planning/ompl_planner/RKPRRTSetup.h"
#include "kinematic_planning/ompl_planner/RKPLazyRRTSetup.h"
#include "kinematic_planning/ompl_planner/RKPSBLSetup.h"
#include "kinematic_planning/ompl_planner/RKPESTSetup.h"
#include "kinematic_planning/ompl_planner/RKPIKSBLSetup.h"
#include "kinematic_planning/ompl_planner/RKPKPIECESetup.h"
#include "kinematic_planning/ompl_planner/RKPIKKPIECESetup.h"

#include <string>
#include <map>

namespace kinematic_planning
{
    
    class RKPModel : public RKPModelBase
    {
    public:
        RKPModel(void) : RKPModelBase()
	{
	}
	
	virtual ~RKPModel(void)
	{
	    for (std::map<std::string, RKPPlannerSetup*>::iterator i = planners.begin(); i != planners.end() ; ++i)
		if (i->second)
		    delete i->second;
	}
	
	void addRRT(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *rrt = new RKPRRTSetup();
	    if (rrt->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[rrt->name] = rrt;
	    else
		delete rrt;
	}
	
	void addLazyRRT(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *rrt = new RKPLazyRRTSetup();
	    if (rrt->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[rrt->name] = rrt;
	    else
		delete rrt;
	}
	
	void addEST(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *est = new RKPESTSetup();
	    if (est->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[est->name] = est;
	    else
		delete est;
	}

	void addSBL(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *sbl = new RKPSBLSetup();
	    if (sbl->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[sbl->name] = sbl;
	    else
		delete sbl;
	}

	void addIKSBL(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *sbl = new RKPIKSBLSetup();
	    if (sbl->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[sbl->name] = sbl;
	    else
		delete sbl;
	}
		
	void addKPIECE(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *kpiece = new RKPKPIECESetup();
	    if (kpiece->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[kpiece->name] = kpiece;
	    else
		delete kpiece;
	}

	
	void addIKKPIECE(std::map<std::string, std::string> &options)
	{
	    RKPPlannerSetup *kpiece = new RKPIKKPIECESetup();
	    if (kpiece->setup(dynamic_cast<RKPModelBase*>(this), options))
		planners[kpiece->name] = kpiece;
	    else
		delete kpiece;
	}

	std::map<std::string, RKPPlannerSetup*> planners;
    };
    
    typedef std::map<std::string, RKPModel*> ModelMap;
    
} // kinematic_planning

#endif

