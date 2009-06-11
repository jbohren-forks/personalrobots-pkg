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

#include "kinematic_planning/ompl_extensions/RKPGoalDefinitions.h"
#include <ros/console.h>

double kinematic_planning::GoalToState::distanceGoal(const ompl::base::State *s) const
{
    const double *vals = static_cast<const ompl::sb::State*>(s)->values;
    for (int i = 0 ; i < dim_; ++i)
	if (bounds_[i].first > vals[i])
	    cVals_[i] = stateVals_[i] + bounds_[i].first - vals[i];
	else
	    if (bounds_[i].second < vals[i])
		cVals_[i] = stateVals_[i] + vals[i] - bounds_[i].second;
	    else
		cVals_[i] = stateVals_[i];
    return static_cast<ompl::sb::SpaceInformation*>(m_si)->distance(static_cast<const ompl::sb::State*>(compState_), static_cast<const ompl::sb::State*>(state));
}

const std::vector< std::pair<double, double> >& kinematic_planning::GoalToState::getBounds(void) const
{
    return bounds_;
}

void kinematic_planning::GoalToState::print(std::ostream &out) const
{
    ompl::sb::GoalState::print(out);
    out << "Joint constraints: " << std::endl;
    for (int i = 0 ; i < dim_ ; ++i)
	out << "[" << bounds_[i].first << ", " << bounds_[i].second << "] ";
    out << std::endl;
}
	
void kinematic_planning::GoalToState::setup(ompl::sb::SpaceInformationKinematic *si, RKPModelBase *model, const std::vector<motion_planning_msgs::JointConstraint> &jc) 
{
    dim_ = si->getStateDimension();
    
    // get the bounds for the state
    bounds_.resize(dim_);
    for (int i = 0 ; i < dim_ ; ++i)
	bounds_[i] = std::make_pair(si->getStateComponent(i).minValue, si->getStateComponent(i).maxValue);

    // tighten the bounds based on the constraints
    for (unsigned int i = 0 ; i < jc.size() ; ++i)
    {
	// get the index at which the joint parameters start
	int idx = model->kmodel->getJointIndexInGroup(jc[i].joint_name, model->groupID);
	if (idx >= 0)
	{
	    unsigned int usedParams = model->kmodel->getJoint(jc[i].joint_name)->usedParams;
	    
	    if (jc[i].min.size() != jc[i].max.size() || jc[i].max.size() != usedParams)
		ROS_ERROR("Constraint on joint %s has incorrect number of parameters. Expected %u", jc[i].joint_name.c_str(), usedParams);
	    else
	    {
		for (unsigned int j = 0 ; j < usedParams ; ++j)
		{
		    if (bounds_[idx + j].first < jc[i].min[j])
			bounds_[idx + j].first = jc[i].min[j];
		    if (bounds_[idx + j].second > jc[i].max[j])
			bounds_[idx + j].second = jc[i].max[j];
		}
	    }   
	}
    }
    
    // check if joint bounds are feasible
    for (int i = 0 ; i < dim_ ; ++i)
	if (bounds_[i].first > bounds_[i].second)
	{
	    ROS_ERROR("Inconsistent set of joint constraints");
	    break;
	}
    
    compState_ = new ompl::sb::State(dim_);
    cVals_ = static_cast<ompl::sb::State*>(compState_)->values;
    
    // compute the middle state
    if (state == NULL)
	state = new ompl::sb::State(dim_);
    stateVals_ = static_cast<ompl::sb::State*>(state)->values;
    
    for (int i = 0 ; i < dim_ ; ++i)
    {
	stateVals_[i] = (bounds_[i].first + bounds_[i].second) / 2.0;
	// increase bounds by epsilon so we do not have errors when checking
	bounds_[i].first  -= 1e-12;
	bounds_[i].second += 1e-12;
    }

    threshold = 1e-12;	    
}

double kinematic_planning::GoalToPosition::distanceGoal(const ompl::base::State *state) const
{
    return evaluateGoalAux(static_cast<const ompl::sb::State*>(state), NULL);
}

bool kinematic_planning::GoalToPosition::isSatisfied(const ompl::base::State *state, double *dist) const
{
    std::vector<bool> decision;
    double d = evaluateGoalAux(static_cast<const ompl::sb::State*>(state), &decision);
    if (dist)
	*dist = d;
    
    for (unsigned int i = 0 ; i < decision.size() ; ++i)
	if (!decision[i])
	    return false;
    return true;
}

void kinematic_planning::GoalToPosition::print(std::ostream &out) const
{
    ompl::sb::GoalRegion::print(out);
    out << "Pose constraints:" << std::endl;
    for (unsigned int i = 0 ; i < pce_.size() ; ++i)
	pce_[i]->print(out);
}
	
double kinematic_planning::GoalToPosition::evaluateGoalAux(const ompl::sb::State *state, std::vector<bool> *decision) const
{
    model_->kmodel->lock();
    update(state);
    
    if (decision)
	decision->resize(pce_.size());
    double distance = 0.0;
    for (unsigned int i = 0 ; i < pce_.size() ; ++i)
    {
	double dPos, dAng;
	pce_[i]->evaluate(&dPos, &dAng);
	if (decision)
	    (*decision)[i] = pce_[i]->decide(dPos, dAng);
	distance += dPos + pce_[i]->getConstraintMessage().orientation_importance * dAng;
    }
    model_->kmodel->unlock();
    
    return distance;
}

void kinematic_planning::GoalToPosition::update(const ompl::sb::State *state) const
{
    model_->kmodel->computeTransformsGroup(state->values, model_->groupID);
    model_->collisionSpace->updateRobotModel();
}    
	
double kinematic_planning::GoalToMultipleConstraints::distanceGoal(const ompl::base::State *s) const
{
    return gp_.distanceGoal(s) + gs_.distanceGoal(s);
}

bool kinematic_planning::GoalToMultipleConstraints::isSatisfied(const ompl::base::State *state, double *dist) const
{
    if (dist)
    {
	double d1, d2;
	bool res1 = gs_.isSatisfied(state, &d1);
	bool res2 = gp_.isSatisfied(state, &d2);
	*dist = d1 + d2;
	return res1 && res2;
    }
    else
	return gs_.isSatisfied(state) && gp_.isSatisfied(state);
}

void kinematic_planning::GoalToMultipleConstraints::sampleNearGoal(ompl::sb::State *s)
{
    sCore_.sampleNear(s, static_cast<ompl::sb::State*>(gs_.state), rho_);
}

void kinematic_planning::GoalToMultipleConstraints::print(std::ostream &out) const
{
    gs_.print(out);
    gp_.print(out);
}

ompl::base::Goal* kinematic_planning::computeGoalFromConstraints(ompl::sb::SpaceInformationKinematic *si, RKPModelBase *model, const motion_planning_msgs::KinematicConstraints &kc)
{
    if (kc.joint.empty() && kc.pose.empty())
    {
	ROS_ERROR("No goal constraints defined. No goal can be selected");
	return NULL;
    }
    
    if (kc.joint.size() > 0 && kc.pose.empty())
	// we have no pose constraints, only joint constraints
	// we compute the 'middle state' and plan towards it
	return new GoalToState(si, model, kc.joint);
    
    if (kc.joint.empty() && kc.pose.size() > 0)
	// we have no joint constraints, only pose constraints
	// we know nothing of the state we want to get to (in terms of joint angles)
	return new GoalToPosition(si, model, kc.pose);
    
    return new GoalToMultipleConstraints(si, model, kc);
}
