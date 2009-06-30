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
#include <planning_environment/kinematic_state_constraint_evaluator.h>

#include "ompl_planning/ModelBase.h"
#include "ompl_planning/extensions/SpaceInformation.h"

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
	}
	
        StateValidityPredicate(SpaceInformationDynamicModel *si, ModelBase *model) : ompl::base::StateValidityChecker()
	{
	    dsi_ = si;
	    ksi_ = NULL;
	    model_ = model;
	}
	
	virtual bool operator()(const ompl::base::State *s) const
	{
	    model_->kmodel->computeTransformsGroup(s->values, model_->groupID);
	    
	    bool valid = kce_.decide(s->values, model_->groupID);
	    if (valid)
	    {
		model_->collisionSpace->updateRobotModel();
		valid = !model_->collisionSpace->isCollision();
	    }
	    
	    return valid;
	}
	
	void setConstraints(const motion_planning_msgs::KinematicConstraints &kc)
	{
	    kce_.clear();
	    kce_.add(model_->kmodel, kc.pose_constraint);

	    // joint constraints simply update the state space bounds
	    if (ksi_)
	    {
		ksi_->clearJointConstraints();
		ksi_->setJointConstraints(kc.joint_constraint);
	    }
	    if (dsi_)
	    {
		dsi_->clearJointConstraints();
		dsi_->setJointConstraints(kc.joint_constraint);
	    }
	}
	
	void clearConstraints(void)
	{
	    kce_.clear();
	    if (ksi_)
		ksi_->clearJointConstraints();
	    if (dsi_)
		dsi_->clearJointConstraints();
	}
	
	const planning_environment::KinematicConstraintEvaluatorSet& getKinematicConstraintEvaluatorSet(void) const
	{
	    return kce_;
	}
	
    protected:
	
	ModelBase                                             *model_;
	
	// one of the next two will be instantiated
	SpaceInformationKinematicModel                        *ksi_;
	SpaceInformationDynamicModel                          *dsi_;

	planning_environment::KinematicConstraintEvaluatorSet  kce_;
    };  
    
} // ompl_planning

#endif
    
