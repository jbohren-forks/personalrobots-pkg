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

#include "ompl_planning/extensions/StateValidator.h"

bool ompl_planning::StateValidityPredicate::operator()(const ompl::base::State *s) const
{
    // for dynamic state spaces, we may get outside bounds
    if (dsi_)
    {
	if (!dsi_->satisfiesBounds(s))
	    return false;
    }
    
    const Clone &c = clones_[position_];
    return check(s, c.em, c.km, c.kce);	    
}

void ompl_planning::StateValidityPredicate::setConstraints(const motion_planning_msgs::KinematicConstraints &kc)
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

void ompl_planning::StateValidityPredicate::clearConstraints(void)
{
    kce_.clear();
    if (ksi_)
	ksi_->clearJointConstraints();
    if (dsi_)
	dsi_->clearJointConstraints();
}

void ompl_planning::StateValidityPredicate::clear(void)
{
    clearConstraints();
    clearClones();
    position_ = 0;
}

void ompl_planning::StateValidityPredicate::printSettings(std::ostream &out) const
{    
    out << "Path constraints:" << std::endl;
    kce_.print(out);
}

bool ompl_planning::StateValidityPredicate::check(const ompl::base::State *s, collision_space::EnvironmentModel *em, planning_models::KinematicModel *km,
						  planning_environment::KinematicConstraintEvaluatorSet *kce) const
{
    km->computeTransformsGroup(s->values, model_->groupID);
    
    bool valid = kce->decide(s->values, model_->groupID);
    if (valid)
    {
	em->updateRobotModel();
	valid = !em->isCollision();
    }
    
    return valid;
}

void ompl_planning::StateValidityPredicate::setupModel(void)
{
    clones_.resize(1);
    clones_[0].km = model_->kmodel;
    clones_[0].em = model_->collisionSpace;
    clones_[0].kce = &kce_;
    position_ = 0;
}

void ompl_planning::StateValidityPredicate::clearClones(void)
{
    for (unsigned int i = 1 ; i < clones_.size() ; ++i)
    {
	delete clones_[i].em; // .km is owned & deleted by .em
	delete clones_[i].kce;		
    }
    clones_.resize(1);
}
