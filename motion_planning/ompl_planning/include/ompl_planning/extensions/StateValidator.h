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

#ifndef OMPL_PLANNING_EXTENSIONS_STATE_VALIDATOR_
#define OMPL_PLANNING_EXTENSIONS_STATE_VALIDATOR_

#include <ompl/base/StateValidityChecker.h>
#include <collision_space/environment.h>
#include <planning_environment/util/kinematic_state_constraint_evaluator.h>

#include "ompl_planning/ModelBase.h"
#include "ompl_planning/extensions/SpaceInformation.h"

#include <iostream>

namespace ompl_planning
{
    
    class StateValidityPredicate : public ompl::base::StateValidityChecker
    {
    public:
        StateValidityPredicate(SpaceInformationKinematicModel *si, ModelBase *model) : ompl::base::StateValidityChecker()
	{
	    ksi_ = si;
	    dsi_ = NULL;
	    model_ = model;
	    setupModel();
	}
	
        StateValidityPredicate(SpaceInformationDynamicModel *si, ModelBase *model) : ompl::base::StateValidityChecker()
	{
	    dsi_ = si;
	    ksi_ = NULL;
	    model_ = model;
	    setupModel();
	}

	virtual ~StateValidityPredicate(void)
	{
	    clearClones();
	}
	
	virtual bool operator()(const ompl::base::State *s) const;
	void setConstraints(const motion_planning_msgs::KinematicConstraints &kc);
	void clearConstraints(void);
	void clear(void);
	void printSettings(std::ostream &out) const;
	
    protected:
	
	bool check(const ompl::base::State *s, collision_space::EnvironmentModel *em, planning_models::KinematicModel *km,
		   planning_environment::KinematicConstraintEvaluatorSet *kce) const;
	void setupModel(void);
	void clearClones(void);
	

	ModelBase                                             *model_;
	
	// one of the next two will be instantiated
	SpaceInformationKinematicModel                        *ksi_;
	SpaceInformationDynamicModel                          *dsi_;
	
	planning_environment::KinematicConstraintEvaluatorSet  kce_;

	
	struct Clone
	{
	    collision_space::EnvironmentModel                     *em;
	    planning_models::KinematicModel                       *km;
	    planning_environment::KinematicConstraintEvaluatorSet *kce;
	};
	
	std::vector<Clone>                                     clones_;
	mutable int                                            position_;
	mutable boost::mutex                                   lock_;
	
	
    };
    
} // ompl_planning

#endif
    
