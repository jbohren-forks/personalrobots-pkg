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

#ifndef KINEMATIC_PLANNING_OMPL_PLANNER_RKP_PLANNER_SETUP_
#define KINEMATIC_PLANNING_OMPL_PLANNER_RKP_PLANNER_SETUP_

#include "kinematic_planning/RKPModelBase.h"
#include "kinematic_planning/RKPStateValidator.h"
#include "kinematic_planning/RKPSpaceInformation.h"
#include "kinematic_planning/RKPProjectionEvaluators.h"
#include "kinematic_planning/RKPDistanceEvaluators.h"

#include <ompl/base/Planner.h>
#include <ompl/extension/samplingbased/kinematic/PathSmootherKinematic.h>
#include <ompl/extension/samplingbased/kinematic/extension/ik/GAIK.h>

#include <ros/console.h>
#include <string>
#include <map>

namespace kinematic_planning
{
    
    class RKPPlannerSetup
    {
    public:
	
	RKPPlannerSetup(void);
	
	virtual ~RKPPlannerSetup(void);
	
	/** For each planner definition, define the set of distance metrics it can use */
	virtual void setupDistanceEvaluators(void);
	
	virtual ompl::ProjectionEvaluator* getProjectionEvaluator(RKPModelBase *model,
								  const std::map<std::string, std::string> &options) const;
	
	virtual void preSetup(RKPModelBase *model, std::map<std::string, std::string> &options);
	virtual void postSetup(RKPModelBase *model, std::map<std::string, std::string> &options);
	
	virtual bool setup(RKPModelBase *model, std::map<std::string, std::string> &options) = 0;
	
	std::string                                                            name;	
	ompl::Planner                                                         *mp;
	ompl::GAIK                                                            *gaik;
	ompl::SpaceInformationKinematic                                       *si;
	ompl::SpaceInformation::StateValidityChecker                          *svc;
	std::map<std::string, ompl::SpaceInformation::StateDistanceEvaluator*> sde;
	ompl::PathSmootherKinematic                                           *smoother;

    protected:
	
	double parseDouble(const std::string &value, double def);
    };

    
} // kinematic_planning

#endif
    
