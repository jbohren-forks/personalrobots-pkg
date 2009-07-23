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

void ompl_planning::StateValidityPredicate::setupModel(ModelBase *model)
{
    model_ = model;
    boost::thread::id id = boost::this_thread::get_id();
    clones_[id].em  = model_->collisionSpace;
    clones_[id].km  = model_->kmodel;
    clones_[id].kce = &kce_;
}

bool ompl_planning::StateValidityPredicate::operator()(const ompl::base::State *s) const
{
    // for dynamic state spaces, we may get outside bounds
    if (dsi_)
    {
	if (!dsi_->satisfiesBounds(s))
	    return false;
    }

    boost::thread::id id = boost::this_thread::get_id();
    
    lock_.lock();
    if (clones_.find(id) == clones_.end())
    {
	ROS_DEBUG("Cloning collision environment");
	Clone &add = clones_[id];
	add.em = model_->collisionSpace->clone();
	add.km = add.em->getRobotModel().get();
	add.kce = new planning_environment::KinematicConstraintEvaluatorSet();
	useConstraints(add.kce, add.km);
    }
    const Clone c = clones_[id];
    lock_.unlock();
    
    return check(s, c.em, c.km, c.kce);
}

void ompl_planning::StateValidityPredicate::setConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kc_ = kc;
    useConstraints(&kce_, model_->kmodel);
    
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

void ompl_planning::StateValidityPredicate::useConstraints(planning_environment::KinematicConstraintEvaluatorSet *kce, planning_models::KinematicModel *km) const
{
    kce->clear();
    kce->add(km, kc_.pose_constraint);
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

void ompl_planning::StateValidityPredicate::clearClones(void)
{
    boost::thread::id id = boost::this_thread::get_id();
    Clone keep = clones_[id];
    for (std::map<boost::thread::id, Clone>::iterator it = clones_.begin() ; it != clones_.end() ; ++it)
    {
	if (it->first == id)
	    continue;
	delete it->second.em;  // .km is owned & deleted by .em
	delete it->second.kce;
    }
    clones_.clear();
    clones_[id] = keep;
}
