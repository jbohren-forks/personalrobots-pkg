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

#include "planning_environment/planning_monitor.h"
#include "planning_environment/kinematic_state_constraint_evaluator.h"

void planning_environment::PlanningMonitor::setPathConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kcPath_ = kc;
    bringConstraintsToModelFrame(kcPath_);
}

void planning_environment::PlanningMonitor::setGoalConstraints(const motion_planning_msgs::KinematicConstraints &kc)
{
    kcGoal_ = kc;
    bringConstraintsToModelFrame(kcPath_);
}

void planning_environment::PlanningMonitor::bringConstraintsToModelFrame(motion_planning_msgs::KinematicConstraints &kc)
{
    for (unsigned int i = 0; i < kc.pose.size() ; ++i)
	tf_->transformPose(getFrameId(), kc.pose[i].pose, kc.pose[i].pose);
}

bool planning_environment::PlanningMonitor::isStateValidOnPath(const planning_models::KinematicModel::StateParams *state) const
{
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();

    // check for collision
    bool valid = !getEnvironmentModel()->isCollision();
    
    if (valid)
    {	    
	KinematicConstraintEvaluatorSet ks;
	ks.add(getKinematicModel(), kcPath_.joint);
	ks.add(getKinematicModel(), kcPath_.pose);
	valid = ks.decide(state->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    return valid;
}

bool planning_environment::PlanningMonitor::isStateValidAtGoal(const planning_models::KinematicModel::StateParams *state) const
{   
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(state->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();
    
    // check for collision
    bool valid = !getEnvironmentModel()->isCollision();
    
    if (valid)
    {	    
	KinematicConstraintEvaluatorSet ks;
	ks.add(getKinematicModel(), kcPath_.joint);
	ks.add(getKinematicModel(), kcPath_.pose);
	ks.add(getKinematicModel(), kcGoal_.joint);
	ks.add(getKinematicModel(), kcGoal_.pose);
	valid = ks.decide(state->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    return valid;    
}

bool planning_environment::PlanningMonitor::isPathValid(const motion_planning_msgs::KinematicPath &path)
{
    planning_models::KinematicModel::StateParams *sp = getKinematicModel()->newStateParams();
    sp->setParams(path.start_state.vals);

    KinematicConstraintEvaluatorSet ks;
    ks.add(getKinematicModel(), kcPath_.joint);
    ks.add(getKinematicModel(), kcPath_.pose);
    
    getEnvironmentModel()->lock();
    getKinematicModel()->lock();
    
    // figure out the poses of the robot model
    getKinematicModel()->computeTransforms(sp->getParams());
    // update the collision space
    getEnvironmentModel()->updateRobotModel();
    
    bool valid = true;
    
    // get the joints this path is for
    std::vector<planning_models::KinematicModel::Joint*> joints(path.names.size());
    for (unsigned int j = 0 ; j < joints.size() ; ++j)
	joints[j] = getKinematicModel()->getJoint(path.names[j]);
    
    // check every state
    for (unsigned int i = 0 ; valid && i < path.states.size() ; ++i)
    {
	unsigned int u = 0;
	for (unsigned int j = 0 ; j < joints.size() ; ++j)
	{
	    /* copy the parameters that describe the joint */
	    std::vector<double> params;
	    for (unsigned int k = 0 ; k < joints[j]->usedParams ; ++k)
		params.push_back(path.states[i].vals[u + k]);
	    u += joints[j]->usedParams;
	    
	    /* set the parameters */
	    sp->setParamsJoint(params, joints[j]->name);
	}
	getKinematicModel()->computeTransforms(sp->getParams());
	getEnvironmentModel()->updateRobotModel();
	
	// check for collision
	valid = !getEnvironmentModel()->isCollision();
    
	// check for validity
	if (valid)
	    valid = ks.decide(sp->getParams());
    }

    // if we got to the last state, we also check the goal constraints
    if (valid)
    {
	ks.add(getKinematicModel(), kcGoal_.joint);
	ks.add(getKinematicModel(), kcGoal_.pose);
	valid = ks.decide(sp->getParams());
    }
    
    getKinematicModel()->unlock();
    getEnvironmentModel()->unlock();
    
    delete sp;
    return valid;
}
